#pragma once
#include <iostream>
#include "rs2Class.hpp"


int main()
try {

	// Pipeline config
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);	// Color
	cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);	// Depth

	// Align color stream to depth stream
	rs2::align align(RS2_STREAM_DEPTH);

	// RsCamera instance
	RsCamera camera(cfg, align);


	// Display image
	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3);
	while (true)
	{
		// カラーフレームを取得
		//camera.getColorFrame(image);

		// Depthフレームを取得
		//camera.getDepthFrame(image);

		// カラーフレームとDepthフレームを並べて取得
		// camera.getComboFrame(image);

		camera.doDeprojectPosition(image);

		// 画像を表示
		cv::imshow("COMBO_FRAME", image);

		// ESCで終了
		if (cv::waitKey(10) == 27) {
			cv::destroyAllWindows();
			break;
		}
	}

	return EXIT_SUCCESS;

}
catch (const rs2::error& e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}