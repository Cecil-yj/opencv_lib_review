#include "cvInclude.h"

// 图片亮区过度曝光，暗区曝光不足。
// 从曝光序列生成和显示HDR图片
// 曝光融合 exposure fusion

void loadExposureSeq(cv::String path, std::vector<cv::Mat>& images, std::vector<float>& times)  // 自动取地址 路径+图片名
{
	path = path + "/";
	std::ifstream list_file((path + "list.txt").c_str());
	std::string name;
	float val;
	while (list_file >> name >> val) {
		cv::Mat img = cv::imread(path + name);
		images.push_back(img);
		times.push_back(1 / val);
	}
	list_file.close();
}

int photo_imaging_fusion(int argc, char **argv)
{
	cv::CommandLineParser parser(argc, argv, "{@input |G:/learn_opencv/images | Input directory that contains images and exposure times. }");
	//! [Load images and exposure times]
	std::vector<cv::Mat> images;
	std::vector<float> times;
	loadExposureSeq(parser.get<cv::String>("@input"), images, times);
	//! [Load images and exposure times]

	//! [Estimate camera response]
	cv::Mat response;
	cv::Ptr<cv::CalibrateDebevec> calibrate = cv::createCalibrateDebevec();
	calibrate->process(images, response, times);
	//! [Estimate camera response]

	//! [Make HDR image]
	cv::Mat hdr;
	cv::Ptr<cv::MergeDebevec> merge_debevec = cv::createMergeDebevec();
	merge_debevec->process(images, hdr, times, response);
	//! [Make HDR image]

	//! [Tonemap HDR image]
	cv::Mat ldr;
	cv::Ptr<cv::Tonemap> tonemap = cv::createTonemap(2.2f);
	tonemap->process(hdr, ldr);
	//! [Tonemap HDR image]

	//! [Perform exposure fusion]
	cv::Mat fusion;
	cv::Ptr<cv::MergeMertens> merge_mertens = cv::createMergeMertens();
	merge_mertens->process(images, fusion);
	//! [Perform exposure fusion]

	//! [Write results]
	cv::imwrite("fusion.png", fusion * 255);
	cv::imwrite("ldr.png", ldr * 255);
	cv::imwrite("hdr.hdr", hdr);
	//! [Write results]

	return 0;
}