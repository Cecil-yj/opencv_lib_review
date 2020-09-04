#ifndef CALLBACK_H
#define CALLBACK_H

#include "cvInclude.h"

void image_watch_demo(); // 01
void loadExposureSeq(cv::String path, std::vector<cv::Mat>& images, std::vector<float>& times);
int photo_imaging_fusion(int argc, char **argv); // 02
void detectAndDisplay(cv::Mat frame);
int face_detect(int argc, const char** argv); // 03



#endif // !CALLBACK_H