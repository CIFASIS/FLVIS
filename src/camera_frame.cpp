#include <include/camera_frame.h>

CameraFrame::CameraFrame()
{

}

void CameraFrame::clear()
{
    T_c_w = SE3(Quaterniond(1,0,0,0),Vec3(0,0,0));
    img.release();
    d_img.release();
    landmarks.clear();
}

void CameraFrame::trackMatchAndEraseOutlier(Mat& newImage,
                                            vector<Vec2>& lm2d_from,
                                            vector<Vec2>& lm2d_to,
                                            vector<Vec2>& outlier)
{
    //STEP1: Track Match ORB
    //STEP2: Computer ORB
    //STEP3: Check Matches
    //STEP4: Update the lm2d and output
    //STEP1:
    vector<Point2f> trackedLM_cvP2f;
    vector<float> err;
    vector<unsigned char> mask_tracked;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    calcOpticalFlowPyrLK(img, newImage,
                         this->get2dPtsVec_cvP2f(), trackedLM_cvP2f,
                         mask_tracked, err, Size(31,31), 3, criteria);

    //STEP2
    vector<unsigned char> mask_hasDescriptor;
    vector<Mat>           trackedLMDescriptors;
    vector<KeyPoint>      trackedLM_cvKP;
    Mat descriptorsMat;
    KeyPoint::convert(trackedLM_cvP2f,trackedLM_cvKP);
    cv::Ptr<DescriptorExtractor> extractor = ORB::create();
    extractor->compute(newImage, trackedLM_cvKP, descriptorsMat);

    for(size_t i=0; i<trackedLM_cvP2f.size(); i++)
    {
        unsigned char hasDescriptor = 0;
        for(size_t j=0; j<trackedLM_cvKP.size(); j++)
        {
            if(trackedLM_cvP2f.at(i).x==trackedLM_cvKP.at(j).pt.x &&
                    trackedLM_cvP2f.at(i).y==trackedLM_cvKP.at(j).pt.y)
            {//has ORB descriptor
                trackedLMDescriptors.push_back(descriptorsMat.row(j));
                hasDescriptor = 1;
                break;
            }
        }
        mask_hasDescriptor.push_back(hasDescriptor);
        if(hasDescriptor==0)
        {
            cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, Scalar(0));
            trackedLMDescriptors.push_back(zeroDescriptor);
        }
    }
    //STEP3
    vector<unsigned char> mask_matchCheck;
    for(size_t i=0; i<trackedLMDescriptors.size(); i++)
    {
        if(norm(landmarks.at(i).lm_descriptor, trackedLMDescriptors.at(i), NORM_HAMMING) <= 100)
        {
            mask_matchCheck.push_back(1);
        }else
        {
            mask_matchCheck.push_back(0);
        }
    }

    //STEP4:
    for(int i=landmarks.size()-1; i>=0; i--)
    {
        if(mask_tracked.at(i)!=1       ||
                mask_hasDescriptor.at(i)!=1 ||
                mask_matchCheck.at(i)!=1)
        {//outliers
            outlier.push_back(this->landmarks.at(i).lm_2d);
            trackedLM_cvP2f.erase(trackedLM_cvP2f.begin()+i);
            this->landmarks.erase(this->landmarks.begin()+i);
        }
    }
    for(size_t i=0; i<landmarks.size(); i++)
    {
        lm2d_from.push_back(landmarks.at(i).lm_2d);
        lm2d_to.push_back(Vec2(trackedLM_cvP2f.at(i).x,trackedLM_cvP2f.at(i).y));
    }
}

void CameraFrame::updateLMT_c_w()
{
    for (auto lm = landmarks.begin(); lm != landmarks.end(); ++lm)
    {
        lm->lm_frame_pose=T_c_w;
    }
}

void CameraFrame::CalReprjInlierOutlier(double &mean_prjerr, vector<Vec2> &outlier, double sh_over_med)
{
    vector<double> distances;
    vector<double> valid_distances;
    for(int i=this->landmarks.size()-1; i>=0; i--)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf())//has depth information
        {
            Vec3 lm3d_w=lm.lm_3d_w;
            Vec3 lm3d_c = DepthCamera::world2cameraT_c_w(lm3d_w,this->T_c_w);
            Vec2 reProj=this->d_camera.camera2pixel(lm3d_c);
            Vec2 lm2d = lm.lm_2d;
            double relativeDist = sqrt(pow(lm2d(0)-reProj(0),2)+pow(lm2d(1)-reProj(1),2));
            distances.push_back(relativeDist);
            if(lm.lm_tracking_state==LM_TRACKING_INLIER){
                valid_distances.push_back(relativeDist);
            }
        }
        else//no depth information
        {
            distances.push_back(0);
        }
    }

    //mean SH
    double sum=0;
    for (size_t i=0; i<valid_distances.size(); i++)
    {
        sum+=valid_distances.at(i);
    }
    mean_prjerr = sum/(double)valid_distances.size();
    //mean SH
    //double sh = mean_prjerr*sh_over_mean;

    //robust MAD SH MAD=1.4826*MED
    sort(valid_distances.begin(), valid_distances.end());
    int half=floor(valid_distances.size()/2);
    double sh = sh_over_med * valid_distances.at(half);
    //cout << "MAD SH=" << sh << endl;
    if(sh>=5.0) sh=5.0;
    for(int i=this->landmarks.size()-1; i>=0; i--)
    {
        if(distances.at(i)>sh)
        {
            outlier.push_back(this->landmarks.at(i).lm_2d);
            this->landmarks.at(i).lm_tracking_state=LM_TRACKING_OUTLIER;
        }else
        {
            this->landmarks.at(i).lm_tracking_state=LM_TRACKING_INLIER;
        }
    }

    //    cout << "-------------------------------------"<<endl << "[";
    //    for(int i=this->landmarks.size()-1; i>=0; i--)
    //    {
    //        cout << distances.at(i)<<" ";
    //    }
    //    cout << "]-------------------------------------"<<endl;
}

void CameraFrame::recover3DPts_c_FromDepthImg(vector<Vec3>& pt3ds,
                                              vector<bool>& maskHas3DInf)
{
    pt3ds.clear();
    maskHas3DInf.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        //use round to find the nearst pixel from depth image;
        Point2f pt=Point2f(round(landmarks.at(i).lm_2d[0]),round(landmarks.at(i).lm_2d[1]));
        Vec3 pt3d;
        //CV_16UC1 = Z16 16-Bit unsigned int
        if(isnan(d_img.at<ushort>(pt)))
        {
            pt3ds.push_back(Vec3(0,0,0));
            maskHas3DInf.push_back(false);
        }
        else
        {
            float z = (d_img.at<ushort>(pt))/d_camera.camera_scale_factor;
            if(z>=0.5&&z<=6.5)
            {
                pt3d[2] = z;
                pt3d[0] = (pt.x - d_camera.camera_cx) * z / d_camera.camera_fx;
                pt3d[1] = (pt.y - d_camera.camera_cy) * z / d_camera.camera_fy;
                pt3ds.push_back(pt3d);
                maskHas3DInf.push_back(true);
            }else
            {
                pt3ds.push_back(Vec3(0,0,0));
                maskHas3DInf.push_back(false);
            }
        }
    }
}

void CameraFrame::recover3DPts_c_FromTriangulation(vector<Vec3> &pt3ds, vector<bool> &maskHas3DInf)
{
    pt3ds.clear();
    maskHas3DInf.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        SE3 T_c_w1=landmarks.at(i).lm_1st_obs_frame_pose;
        Vec3 baseline = T_c_w1.translation()-T_c_w.translation();
        double baseline_dis = sqrt(pow(baseline[0],2)+pow(baseline[1],2)+pow(baseline[2],2));
        if(baseline_dis>=1.0)
        {
            Vec3 pt3d_w = Triangulation::triangulationPt(landmarks.at(i).lm_1st_obs_2d,landmarks.at(i).lm_2d,
                                                         landmarks.at(i).lm_1st_obs_frame_pose,T_c_w,
                                                         d_camera.camera_fx,
                                                         d_camera.camera_fy,
                                                         d_camera.camera_cx,
                                                         d_camera.camera_cy);
            Vec3 pt3d_c = DepthCamera::world2cameraT_c_w(pt3d_w,T_c_w);
            if(pt3d_c[2]>=0.5 && pt3d_c[2]<=15)
            {
                pt3ds.push_back(pt3d_c);
                maskHas3DInf.push_back(true);
            }
            else
            {
                pt3ds.push_back(Vec3(0,0,0));
                maskHas3DInf.push_back(false);
            }
        }
        else
        {
            pt3ds.push_back(Vec3(0,0,0));
            maskHas3DInf.push_back(false);
        }
    }
}
void CameraFrame::depthInnovation(void)
{
    vector<Vec3> pts3d_c_depth_measurement;
    vector<Vec3> pts3d_c_triangulation;
    vector<bool> depth_measurement_mask;
    vector<bool> triangulation_mask;
    this->recover3DPts_c_FromTriangulation(pts3d_c_triangulation,triangulation_mask);
    this->recover3DPts_c_FromDepthImg(pts3d_c_depth_measurement,depth_measurement_mask);
    for(size_t i=0; i<landmarks.size(); i++)
    {
        if(depth_measurement_mask.at(i)==false && triangulation_mask.at(i)==false) continue;
        Vec3 lm_c_measure;
        if(depth_measurement_mask.at(i)==true && triangulation_mask.at(i)==true)
        {
            lm_c_measure = 0.5*(pts3d_c_triangulation.at(i)+pts3d_c_depth_measurement.at(i));
        }else if(depth_measurement_mask.at(i)==true && triangulation_mask.at(i)==false)
        {
            lm_c_measure = pts3d_c_depth_measurement.at(i);
        }else if(depth_measurement_mask.at(i)==false && triangulation_mask.at(i)==true)
        {
            lm_c_measure = pts3d_c_triangulation.at(i);
        }

        if(landmarks.at(i).hasDepthInf())
        {
            //transfor to Camera frame
            Vec3 lm_c = DepthCamera::world2cameraT_c_w(landmarks.at(i).lm_3d_w,this->T_c_w);
            //apply IIR Filter
            Vec3 lm_c_update = lm_c*0.8+lm_c_measure*0.2;
            //update to world frame
            landmarks.at(i).lm_3d_w = DepthCamera::camera2worldT_c_w(lm_c_update,this->T_c_w);
            landmarks.at(i).lmState = LMSTATE_NORMAL;
        }
        else//Do not have position
        {
            Vec3 pt3d_w = DepthCamera::camera2worldT_c_w(lm_c_measure,this->T_c_w);
            landmarks.at(i).lm_3d_w = pt3d_w;
            landmarks.at(i).lmState = LMSTATE_NORMAL;
            landmarks.at(i).lm_has_3d = true;
        }

    }
}


void CameraFrame::getValid2d3dPair_cvPf(vector<Point2f> &p2d, vector<Point3f> &p3d)
{
    p2d.clear();
    p3d.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf() && lm.lm_tracking_state==LM_TRACKING_INLIER)
        {
            p2d.push_back(Point2f(lm.lm_2d[0],lm.lm_2d[1]));
            p3d.push_back(Point3f(lm.lm_3d_w[0],lm.lm_3d_w[1],lm.lm_3d_w[2]));
        }
    }
}

void CameraFrame::unpack(vector<Vec2> &pt2d,
                         vector<Mat>  &descriptors,
                         vector<Vec3> &pt3d,
                         vector<unsigned char> &mask3d)
{
    pt2d.clear();
    descriptors.clear();
    pt3d.clear();
    mask3d.clear();
    for(size_t i=0; i<pt2d.size(); i++)
    {
        pt2d.push_back(landmarks.at(i).lm_2d);
        pt3d.push_back(landmarks.at(i).lm_3d_w);
        descriptors.push_back(landmarks.at(i).lm_descriptor);
        if(landmarks.at(i).hasDepthInf())
        {mask3d.push_back(1);}
        else
        {mask3d.push_back(0);}
    }
}

vector<Vec3> CameraFrame::getValid3dPts(void)
{
    vector<Vec3> ret;
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf())
        {
            ret.push_back(lm.lm_3d_w);
        }
    }
    return ret;

}

vector<Point2f> CameraFrame::get2dPtsVec_cvP2f(void)
{
    vector<Point2f> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(Point2f(landmarks.at(i).lm_2d[0],
                      landmarks.at(i).lm_2d[1]));
    }
    return ret;
}
vector<Point3f> CameraFrame::get3dPtsVec_cvP3f(void)
{
    vector<Point3f> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(Point3f(landmarks.at(i).lm_2d[0],
                      landmarks.at(i).lm_2d[1],
                landmarks.at(i).lm_2d[3]));
    }
    return ret;
}

vector<Vec2> CameraFrame::get2dPtsVec(void)
{
    vector<Vec2> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(landmarks.at(i).lm_2d);
    }
    return ret;
}

vector<Vec3> CameraFrame::get3dPtsVec(void)
{
    vector<Vec3> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(landmarks.at(i).lm_3d_w);
    }
    return ret;
}

vector<Mat>  CameraFrame::getDescriptorVec(void)
{
    vector<Mat> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(landmarks.at(i).lm_descriptor);
    }
    return ret;
}

void CameraFrame::getKeyFrameInf(vector<uint64_t> &lm_id, vector<Vec2> &lm_2d, vector<Vec3> &lm_3d, vector<Mat> &lm_descriptors)
{
    lm_id.clear();
    lm_2d.clear();
    lm_3d.clear();
    lm_descriptors.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf())
        {
            lm_3d.push_back(lm.lm_3d_w);
            lm_2d.push_back(lm.lm_2d);
            lm_id.push_back(lm.lm_id);
            lm_descriptors.push_back(lm.lm_descriptor);
        }
    }

}
