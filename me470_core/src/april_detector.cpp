#include "../include/april_detector.h"

#define Focus 630
#define Cart_Length 50
using namespace cv;

april_detector::april_detector()
{
    // default class as 36h11
    aprilopt=tag36h11_create();
    april_det_opt=apriltag_detector_create();
    apriltag_detector_add_family(april_det_opt,aprilopt);
    
    //default parameters
    april_det_opt->quad_decimate=1.0;
    april_det_opt->quad_sigma=0.0;
    april_det_opt->nthreads=4;
    april_det_opt->debug=0;
    april_det_opt->refine_edges=0;
    april_det_opt->refine_decode=0;
    april_det_opt->refine_pose=0;
    std::cout<<"Default Apriltag detector configuration set"<<std::endl;

    cv::namedWindow("April tags Detecion");
}

april_detector::~april_detector()
{
    if(aprilopt!=NULL)
        tag36h10_destroy(aprilopt);
    
    if(april_det_opt!=NULL)
        apriltag_detector_destroy(april_det_opt);

    cv::destroyWindow("April tags Detecion");
}

void april_detector::detection(cv::Mat & gray)
{
    image_u8_t im = { .width = gray.cols,
    .height = gray.rows,
    .stride = gray.cols,
    .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(april_det_opt, &im);
    //std::cout << zarray_size(detections) << " tags detected" << std::endl;

    detection_show(detections,gray);
    zarray_destroy(detections);
}

float april_detector::detection_distance(cv::Mat & gray, int target_id)
{
    image_u8_t im = { .width = gray.cols,
    .height = gray.rows,
    .stride = gray.cols,
    .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(april_det_opt, &im);
    //std::cout << zarray_size(detections) << " tags detected" << std::endl;
    float distance=-1;

    apriltag_detection_t *target_det=NULL;
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if((det->id)==target_id){
            zarray_get(detections, i, &target_det);
            break;
        }
    }
    if(target_det!=NULL)
    {
        float pixel_difference=sqrt(1.0*(((target_det->p[3][0])-(target_det->p[0][0]))*((target_det->p[3][0])-(target_det->p[0][0]))+
            ((target_det->p[3][1])-(target_det->p[0][1]))*((target_det->p[3][1])-(target_det->p[0][1]))));
        distance = (Focus*16.0)/pixel_difference-Cart_Length;
    }else
    {
        std::cout<<"No target tag detected"<<std::endl;
    }
    //detection_show(detections,gray);
    zarray_destroy(detections);

    return distance;
}

void april_detector::detection_show(zarray_t * detections,cv::Mat & frame)
{
     for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            std::stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }

        imshow("April tags Detecion", frame);
        waitKey(30);
}