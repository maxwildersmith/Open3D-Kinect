// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>


#include "Open3D/Open3D.h"
#include "Open3D/Registration/ColoredICP.h"
#include "Open3D/Registration/FastGlobalRegistration.h"
#include "Open3D/IO/Sensor/AzureKinect/K4aPlugin.h"
#include "Open3D/Geometry/TetraMesh.h"
using namespace open3d;
using namespace std;
using namespace open3d::geometry;

Eigen::Matrix4d rot;


shared_ptr<PointCloud> CapturePointCloud(io::AzureKinectSensor* sensor, camera::PinholeCameraIntrinsic* intrin, shared_ptr<RGBDImage> rgbd)
{
	//while (true) {
	rgbd = sensor->CaptureFrame(true);
	if (rgbd == nullptr) {
		std::cout << " NULL CAPTURED!!!!!!!" << std::endl;
		//continue;
	}
	//break;
//}

	rgbd->depth_ = *(rgbd->depth_.ConvertDepthToFloatImage(1000, 10));

	return PointCloud::CreateFromRGBDImage(*rgbd, *intrin, rot, true);
}

// A simplified version of examples/Cpp/Visualizer.cpp to demonstrate linking
// an external project to Open3D.
int main(int argc, char* argv[]) {
	using namespace open3d;

	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
	if (argc < 3) {
		utility::LogInfo("Open3D {}\n", OPEN3D_VERSION);
		utility::LogInfo("\n");
		utility::LogInfo("Usage:\n");
		utility::LogInfo("    > TestVisualizer [k4a|mesh|pointcloud] [filename]\n");
		// CI will execute this file without input files, return 0 to pass
		return 0;
	}

	rot << 1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;

	std::string option(argv[1]);
	if (option == "mesh") {
		auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
		if (io::ReadTriangleMesh(argv[2], *mesh_ptr)) {
			utility::LogInfo("Successfully read {}\n", argv[2]);
		}
		else {
			utility::LogWarning("Failed to read {}\n\n", argv[2]);
			return 1;
		}
		mesh_ptr->ComputeVertexNormals();
		visualization::DrawGeometries({ mesh_ptr }, "Mesh", 1600, 900);
	}
	else if (option == "pointcloud") {
		auto cloud_ptr = std::make_shared<geometry::PointCloud>();
		if (io::ReadPointCloud(argv[2], *cloud_ptr)) {
			utility::LogInfo("Successfully read {}\n", argv[2]);
		}
		else {
			utility::LogWarning("Failed to read {}\n\n", argv[2]);
			return 1;
		}
		cloud_ptr->NormalizeNormals();
		visualization::DrawGeometries({ cloud_ptr }, "PointCloud", 1600, 900);
	}
	else if (option == "record") {

	}
	else if (option == "reconstruct") { //Option to reconstruct a 3d object from different mkv files
		std::string mkv_filename1 = utility::GetProgramOptionAsString(argc, argv, "--file1");
		std::string mkv_filename2 = utility::GetProgramOptionAsString(argc, argv, "--file2");


		k4a_playback_t playback_handle1 = NULL;
		k4a_playback_t playback_handle2 = NULL;

		if (k4a_playback_open(mkv_filename1.c_str(), &playback_handle1) != K4A_RESULT_SUCCEEDED
			|| k4a_playback_open(mkv_filename1.c_str(), &playback_handle2) != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to open recordings\n");
			return 1;
		}

		printf("Recording1 is %lld seconds long, recording 2 is %11d seconds long\n",
			k4a_playback_get_last_timestamp_usec(playback_handle1) / 1000000,
			k4a_playback_get_last_timestamp_usec(playback_handle2) / 1000000);

		k4a_capture_t capture = NULL;
		k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
		k4a_calibration_t cal1;
		k4a_calibration_t cal2;
		_k4a_transformation_t* transformation1;
		_k4a_transformation_t* transformation2;


		if (K4A_RESULT_SUCCEEDED !=
			k4a_playback_get_calibration(playback_handle1, &cal1)) {
			utility::LogError("Failed to get calibration 1");
		}


		transformation1 = k4a_transformation_create(&cal1);
		if (K4A_RESULT_SUCCEEDED !=
			k4a_playback_get_calibration(playback_handle1, &cal2)) {
			utility::LogError("Failed to get calibration 2");
		}
		transformation2 = k4a_transformation_create(&cal2);


		bool flag_exit = false;
		bool flag_play = true;
		visualization::VisualizerWithKeyCallback vis;
		vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
			[&](visualization::Visualizer* vis) {
			flag_exit = true;
			return true;
		});
		vis.RegisterKeyCallback(
			GLFW_KEY_SPACE, [&](visualization::Visualizer* vis) {
			if (flag_play) {
				utility::LogInfo(
					"Playback paused, press [SPACE] to continue");
			}
			else {
				utility::LogInfo(
					"Playback resumed, press [SPACE] to pause");
			}
			flag_play = !flag_play;
			return true;
		});

		vis.CreateVisualizerWindow("Open3D Azure Kinect MKV player", 1920, 540);
		utility::LogInfo(
			"Starting to play. Press [SPACE] to pause. Press [ESC] to "
			"exit.");

		bool is_geometry_added = false;
		std::shared_ptr<geometry::PointCloud> pointcloud_ptr(
			new geometry::PointCloud);

		std::shared_ptr<geometry::PointCloud> pointcloud_ptr2(
			new geometry::PointCloud);

		Eigen::Matrix3d rot;
		rot << 1, 0, 0,
			0, -1, 0,
			0, 0, -1;

		while (result == K4A_STREAM_RESULT_SUCCEEDED && !flag_exit)
		{
			if (flag_play) {
				result = k4a_playback_get_next_capture(playback_handle1, &capture);
				if (result == K4A_STREAM_RESULT_SUCCEEDED)
				{
					try {
						auto rgbd = io::AzureKinectSensor::DecompressCapture(capture, transformation1);
						std::cout << rgbd->IsEmpty() << std::endl;
						auto x = rgbd->depth_.data_;
						std::cout << *std::min_element(x.begin(), x.end()) << std::endl;
						std::cout << *std::max_element(x.begin(), x.end()) << std::endl;
					}
					catch (const std::exception e) {
						std::cout << "asdfadsfasdf      " << e.what() << std::endl;
					}
					k4a_image_t k4a_depth = k4a_capture_get_depth_image(capture);
					geometry::Image depth;
					int width = k4a_image_get_width_pixels(k4a_depth);
					int height = k4a_image_get_height_pixels(k4a_depth);

					k4a_image_t k4a_transformed_depth = nullptr;


					depth.Prepare(width, height, 1, sizeof(uint16_t));
					memcpy(depth.data_.data(), k4a_image_get_buffer(k4a_depth), k4a_image_get_size(k4a_depth));

					pointcloud_ptr->points_ = (geometry::PointCloud::CreateFromDepthImage(depth, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault, Eigen::Matrix4d::Identity(), 1000, 10, 1, true))->points_;
					pointcloud_ptr->Rotate(rot, false);
					k4a_image_release(k4a_depth);
					vis.AddGeometry(pointcloud_ptr);

				}
				else if (result == K4A_STREAM_RESULT_EOF)
				{
					break;
				}
			}
			vis.UpdateGeometry();
			vis.PollEvents();
			vis.UpdateRender();
		}
		if (result == K4A_STREAM_RESULT_FAILED)
		{
			printf("Failed to read entire recording\n");
			return 1;
		}

		k4a_playback_close(playback_handle1);
		k4a_playback_close(playback_handle2);

		/*
		io::MKVReader mkv_reader;
		mkv_reader.Open(mkv_filename1);
		if (!mkv_reader.IsOpened()) {
			utility::LogError("Error opening file");
			return -1;
		}
		std::cout << "color mode: " << mkv_reader.GetMetadata().color_mode_ << "  depth mode: " << mkv_reader.GetMetadata().depth_mode_ << std::endl;
		bool flag_exit = false;
		bool flag_play = true;
		visualization::VisualizerWithKeyCallback vis;
		vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
			[&](visualization::Visualizer* vis) {
			flag_exit = true;
			return true;
		});
		vis.RegisterKeyCallback(
			GLFW_KEY_SPACE, [&](visualization::Visualizer* vis) {
			if (flag_play) {
				utility::LogInfo(
					"Playback paused, press [SPACE] to continue");
			}
			else {
				utility::LogInfo(
					"Playback resumed, press [SPACE] to pause");
			}
			flag_play = !flag_play;
			return true;
		});

		vis.CreateVisualizerWindow("Open3D Azure Kinect MKV player", 1920, 540);
		utility::LogInfo(
			"Starting to play. Press [SPACE] to pause. Press [ESC] to "
			"exit.");

		bool is_geometry_added = false;
		int idx = 0;
		while (!mkv_reader.IsEOF() && !flag_exit) {
			if (flag_play) {
				auto im_rgbd = mkv_reader.NextFrame();
				if (im_rgbd == nullptr) continue;



				if (!is_geometry_added) {
					vis.AddGeometry(im_rgbd);
					is_geometry_added = true;
				}
			}

			vis.UpdateGeometry();
			vis.PollEvents();
			vis.UpdateRender();
		}

		mkv_reader.Close();
		*/

	}
	else if (option == "test") {
		int device_count = k4a_device_get_installed_count();
		if (device_count == 0) {
			utility::LogInfo("No kinects detected!\n");
			return -1;
		}
		utility::LogInfo("{} devices connected", device_count);

		camera::PinholeCameraIntrinsic intrin;
		vector<camera::PinholeCameraIntrinsic> intrins;


		std::unordered_map<std::string, std::string> config{
			{"color_format", "K4A_IMAGE_FORMAT_COLOR_MJPG"},
			{"color_resolution", "K4A_COLOR_RESOLUTION_3072P"},
			{"depth_mode", "K4A_DEPTH_MODE_NFOV_2X2BINNED"},
			{"camera_fps", "K4A_FRAMES_PER_SECOND_15"},
			{"synchronized_images_only", "true"},
			{"depth_delay_off_color_usec", "0"},
			{"wired_sync_mode", "K4A_WIRED_SYNC_MODE_STANDALONE"},
			{"subordinate_delay_off_master_usec", "0"},
			{"disable_streaming_indicator", "false"},
		};

		io::AzureKinectSensorConfig sensor_config(config);

		k4a_device_t sensor1;
		k4a_device_open(0, &sensor1);
		k4a_calibration_t cal1;
		k4a_device_get_calibration(sensor1, K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_COLOR_RESOLUTION_3072P, &cal1);
		k4a_device_close(sensor1);
		// Use color camera intrinsics, 600's expect last is 300's
		//intrin.SetIntrinsics(1280, 720, cal1.depth_camera_calibration.intrinsics.parameters.v[2], cal1.depth_camera_calibration.intrinsics.parameters.v[3], cal1.depth_camera_calibration.intrinsics.parameters.v[0], cal1.depth_camera_calibration.intrinsics.parameters.v[1]);

		//intrin.SetIntrinsics(1920, 1080, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]);
		intrins.push_back(camera::PinholeCameraIntrinsic(4096, 3072, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]));
		cout << "Intrinsics: 1920, 1440, " << cal1.color_camera_calibration.intrinsics.parameters.v[2] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[3] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[0] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[1] << endl;

		io::AzureKinectSensor sensor(sensor_config);


		if (!sensor.Connect(0)) {
			utility::LogWarning("Failed to connect to sensor, abort.");
			return 1;
		}

		int scalVal = 1;
		bool flag_exit = false;
		bool geo_added = false;

		visualization::VisualizerWithKeyCallback visualizer;

		visualizer.RegisterKeyCallback(GLFW_KEY_ESCAPE, [&](visualization::Visualizer* visualizer) {
			flag_exit = true;
			return false;
		});


		visualizer.CreateVisualizerWindow("3D Reconstruction Pipeline", 1920, 1080);



		shared_ptr<RGBDImage> rgbd;

		rgbd = sensor.CaptureFrame(true);
		if (rgbd == nullptr) {
			std::cout << " NULL CAPTURED!!!!!!!" << std::endl;
			//continue;
		}

		visualizer.AddGeometry(rgbd);


		do {
			visualizer.PollEvents();
			visualizer.UpdateRender();

			//pointcloud = *pointcloud.CreateFromRGBDImage(*rgbd, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault);
			//PrintPointCloud(pointcloud);
		} while (!flag_exit);

		utility::LogInfo("Finished");
	}
	else if (option == "k4a") { //*******************************************************
		int device_count = k4a_device_get_installed_count();
		if (device_count == 0) {
			utility::LogInfo("No kinects detected!\n");
			return -1;
		}
		utility::LogInfo("{} devices connected", device_count);

		camera::PinholeCameraIntrinsic intrin;
		camera::PinholeCameraIntrinsic intrin2;
		vector<camera::PinholeCameraIntrinsic> intrins;


		std::unordered_map<std::string, std::string> config{
			{"color_format", "K4A_IMAGE_FORMAT_COLOR_MJPG"},
			{"color_resolution", "K4A_COLOR_RESOLUTION_3072P"},
			{"depth_mode", "K4A_DEPTH_MODE_NFOV_2X2BINNED"},
			{"camera_fps", "K4A_FRAMES_PER_SECOND_15"},
			{"synchronized_images_only", "true"},
			{"depth_delay_off_color_usec", "0"},
			{"wired_sync_mode", "K4A_WIRED_SYNC_MODE_STANDALONE"},
			{"subordinate_delay_off_master_usec", "0"},
			{"disable_streaming_indicator", "false"},
		};

		io::AzureKinectSensorConfig sensor_config(config);

		k4a_device_t sensor1;
		k4a_device_open(0, &sensor1);
		k4a_calibration_t cal1;
		k4a_device_get_calibration(sensor1, K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_COLOR_RESOLUTION_3072P, &cal1);
		k4a_device_close(sensor1);
		// Use color camera intrinsics, 600's expect last is 300's
		//intrin.SetIntrinsics(1280, 720, cal1.depth_camera_calibration.intrinsics.parameters.v[2], cal1.depth_camera_calibration.intrinsics.parameters.v[3], cal1.depth_camera_calibration.intrinsics.parameters.v[0], cal1.depth_camera_calibration.intrinsics.parameters.v[1]);

		//intrin.SetIntrinsics(1920, 1080, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]);
		intrins.push_back(camera::PinholeCameraIntrinsic(4096, 3072, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]));
		cout << "Intrinsics: 1920, 1440, " << cal1.color_camera_calibration.intrinsics.parameters.v[2] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[3] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[0] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[1] << endl;

		io::AzureKinectSensor sensor2(sensor_config);

		if (device_count == 2) {
			k4a_device_open(1, &sensor1);
			k4a_device_get_calibration(sensor1, K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_COLOR_RESOLUTION_3072P, &cal1);
			//intrin2.SetIntrinsics(1920, 1080, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]);
			intrins.push_back(camera::PinholeCameraIntrinsic(4096, 3072, cal1.color_camera_calibration.intrinsics.parameters.v[2], cal1.color_camera_calibration.intrinsics.parameters.v[3], cal1.color_camera_calibration.intrinsics.parameters.v[0], cal1.color_camera_calibration.intrinsics.parameters.v[1]));

			cout << "Intrinsics: 1280, 720, " << cal1.color_camera_calibration.intrinsics.parameters.v[2] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[3] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[0] << ", " << cal1.color_camera_calibration.intrinsics.parameters.v[1] << endl;
			k4a_device_close(sensor1);


			if (!sensor2.Connect(1)) {
				utility::LogWarning("Failed to connect to sensor 2, abort.");
				return 1;
			}
		}

		io::AzureKinectSensor sensor(sensor_config);


		if (!sensor.Connect(0)) {
			utility::LogWarning("Failed to connect to sensor, abort.");
			return 1;
		}

		int scalVal = 1;
		bool flag_exit = false;
		bool geo_added = false;

		visualization::VisualizerWithKeyCallback visualizer;

		visualizer.RegisterKeyCallback(GLFW_KEY_ESCAPE, [&](visualization::Visualizer* visualizer) {
			flag_exit = true;
			return false;
		});


		visualizer.CreateVisualizerWindow("3D Reconstruction Pipeline", 1920, 1080);

		std::shared_ptr<geometry::PointCloud> pointcloud_ptr(
			new geometry::PointCloud);

		std::shared_ptr<geometry::PointCloud> pointcloud_ptr2(
			new geometry::PointCloud);

		std::shared_ptr<geometry::PointCloud> pc_merge(
			new geometry::PointCloud);



		shared_ptr<RGBDImage> rgbd;

		Eigen::Vector3d transform;
		transform << 2, 0, 0;

		float speed = .01;

		Eigen::Matrix4d scale_up;

		scale_up << 2, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 2, 0,
			0, 0, 0, 1;

		Eigen::Matrix4d scale_down;

		scale_down << .5, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, .5, 0,
			0, 0, 0, 1;

		Eigen::Vector3d forward_move;
		forward_move << 0, 0, -speed;

		Eigen::Vector3d back_move;
		back_move << 0, 0, speed;

		Eigen::Vector3d left_move;
		left_move << -speed, 0, 0;

		Eigen::Vector3d right_move;
		right_move << speed, 0, 0;

		Eigen::Vector3d up_move;
		up_move << 0, speed, 0;

		Eigen::Vector3d down_move;
		down_move << 0, -speed, 0;

		double xy_scale;
		double z_scale;

		/*
		while (true) {
			rgbd = sensor.CaptureFrame(true);

			if (rgbd == nullptr) {
				std::cout << " NULL CAPTURED!!!!!!!" << std::endl;
				continue;
				//return 1;
			}
			break;
		}
		rgbd->depth_ = *(rgbd->depth_.ConvertDepthToFloatImage(1000, 10));

		*/
		/*
		Eigen::Matrix4d scal;
		scal << 1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, -1, 0,
			0, 0, 0, 1;

		while (true) {
			rgbd = sensor.CaptureFrame(true);

			if (rgbd == nullptr) {
				std::cout << " NULL CAPTURED!!!!!!!" << std::endl;
				continue;
				//return 1;
			}
			break;
		}
		//auto x = rgbd->depth_.data_;
		//for (int i=0;i<50;++i)
		 //   printf("\\x%.2x", x[i]);
		std::cout << std::endl << "Before change: " << (int) rgbd->depth_.data_.at(100)<<"  "<< (int)rgbd->depth_.data_.at(101) << std::endl;
		std::cout << "Before:  depth bytes: " << rgbd->depth_.bytes_per_channel_ << " num channels: " << rgbd->depth_.num_of_channels_ << std::endl;

		try {

			geometry::Image d = rgbd->depth_;
			//geometry::Image d2;
			//d2.Prepare(d.width_, d.height_, 1, 4);
			//std::cout << d2.BytesPerLine() << "  d: " << d.BytesPerLine() << std::endl;


			//for (int i = 0; i < d.data_.size(); i+=2) {
			//    d2.data_[4 * i] = 0;
			//    d2.data_[4 * i + 1] = 0;
			//    d2.data_[4 * i + 2] = d.data_[i];
			//    d2.data_[4 * i + 3] = d.data_[i+1];
			//}
			//for (int x = 0; x < d2.width_; ++x)
			//    for (int y = 0; y < d2.height_; ++y)
			//        *(d2.PointerAt<uint16_t>(x, y*2)) = x * y * 100;// *d.PointerAt<uint16_t>(x, y) * 5;

			//rgbd = geometry::RGBDImage::CreateFromColorAndDepth(rgbd->color_, rgbd->depth_);
			rgbd->depth_ = *d.ConvertDepthToFloatImage(1000, 10);

			//rgbd->depth_ = d2;
			std::cout << std::endl << "After change: " << (int)rgbd->depth_.data_.at(100) << "  " << (int)rgbd->depth_.data_.at(101) << std::endl;

		std::cout << "After:  depth bytes: " << rgbd->depth_.bytes_per_channel_ << " num channels: " << rgbd->depth_.num_of_channels_ << std::endl;

		//x = rgbd->depth_.data_;
		//for (int i = 0; i < 50; ++i)
		//    printf("\\x%.2x", x[i]);

		std::cout << "color res: " << rgbd->color_.width_ << "x" << rgbd->color_.height_ << "  depth res: " << rgbd->depth_.width_ << "x" << rgbd->depth_.height_ << std::endl;
		std::cout << "color bytes: " << rgbd->color_.bytes_per_channel_ << " num channels: " << rgbd->color_.num_of_channels_ << "  depth bytes: " << rgbd->depth_.bytes_per_channel_ << " num channels: " << rgbd->depth_.num_of_channels_ << std::endl;


			cout << pointcloud_ptr->GetMaxBound() << endl;
			//pointcloud_ptr = (geometry::PointCloud::CreateFromDepthImage(rgbd->depth_, intrin, Eigen::Matrix4d::Identity(), 1000, 1000, 1, true));
			//pointcloud_ptr->Rotate(rot, false);

			//pointcloud_ptr->EstimateNormals();
			//auto mesh = geometry::TriangleMesh::CreateFromPointCloudPoisson(*pointcloud_ptr);
			//pointcloud_ptr->Translate(-transform);
			//visualizer.AddGeometry(std::get<0>(mesh));

			visualizer.AddGeometry(pointcloud_ptr);
			//visualizer.AddGeometry(rgbd);
			visualizer.UpdateGeometry();
			visualizer.PollEvents();
			visualizer.UpdateRender();
		}
		catch (const std::exception & e) {
			std::cout << "asfdsdf" << e.what() << std::endl;
		}
		//pointcloud_ptr->EstimateNormals();
		//pointcloud_ptr->NormalizeNormals();

		*/
		//rgbd = CapturePointCloud(&sensor, &intrin, pointcloud_ptr);
		pointcloud_ptr = CapturePointCloud(&sensor, &intrins.front(), rgbd); //PointCloud::CreateFromRGBDImage(*rgbd, intrin, rot, true);
		visualizer.AddGeometry(pointcloud_ptr);

		//std::cout << "PtCloud 1 captured and displayed" << std::endl;

		//CapturePointCloud(&sensor, &intrin, pointcloud_ptr);
		//visualizer.AddGeometry(pointcloud_ptr);
		//visualizer.UpdateGeometry();
		visualizer.AddGeometry(pointcloud_ptr2);
		visualizer.UpdateGeometry();

		shared_ptr<OrientedBoundingBox> crop_box(new OrientedBoundingBox());


		visualizer.AddGeometry(crop_box);

		visualizer.RegisterKeyCallback(GLFW_KEY_Q, [&](visualization::Visualizer* visualizer) {
			if (device_count == 2)
				*pointcloud_ptr2 = *CapturePointCloud(&sensor2, &intrins.back(), rgbd);
			else
				*pointcloud_ptr2 = *CapturePointCloud(&sensor, &intrins.front(), rgbd);
			cout << "pcl2" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		Eigen::Matrix4d transformMat = Eigen::Matrix4d::Identity();


		visualizer.RegisterKeyCallback(GLFW_KEY_W, [&](visualization::Visualizer* visualizer) {
			if (device_count == 2) {
				*pointcloud_ptr2 = *CapturePointCloud(&sensor2, &intrins.back(), rgbd);
				*pointcloud_ptr = *CapturePointCloud(&sensor, &intrins.front(), rgbd);
				cout << "using " << transformMat << endl;
				pointcloud_ptr->Transform(transformMat);
			}
			else {
				*pointcloud_ptr = *CapturePointCloud(&sensor, &intrins.front(), rgbd);
			}
			cout << "Updated both" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_D, [&](visualization::Visualizer* visualizer) {
			if (device_count == 2)
				*pointcloud_ptr2 = *pointcloud_ptr2->VoxelDownSample(.002);

			*pointcloud_ptr = *pointcloud_ptr->VoxelDownSample(.002);
			cout << "downsampled pointclouds" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_X, [&](visualization::Visualizer* visualizer) {
			if (device_count == 2)
				*pointcloud_ptr2 = *pointcloud_ptr2->VoxelDownSample(.002);

			*pointcloud_ptr = *pointcloud_ptr->VoxelDownSample(.002);
			cout << "downsampled pointclouds" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});


		visualizer.RegisterKeyCallback(GLFW_KEY_B, [&](visualization::Visualizer* visualizer) {
			if (device_count == 2)
				*crop_box = *(new OrientedBoundingBox(pointcloud_ptr2->GetAxisAlignedBoundingBox().GetOrientedBoundingBox()));
			else
				*crop_box = *(new OrientedBoundingBox(pointcloud_ptr2->GetAxisAlignedBoundingBox().GetOrientedBoundingBox()));

			xy_scale = crop_box->extent_.x();
			z_scale = crop_box->extent_.y();
			cout << "Made Bounding Box" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});


		visualizer.RegisterKeyCallback(GLFW_KEY_MINUS, [&](visualization::Visualizer* visualizer) {
			xy_scale -= .1;
			crop_box->extent_ << xy_scale, z_scale, xy_scale;

			cout << "rescaled" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_EQUAL, [&](visualization::Visualizer* visualizer) {
			xy_scale += .1;
			crop_box->extent_ << xy_scale, z_scale, xy_scale;

			cout << "rescaled" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_0, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(up_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_9, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(down_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_UP, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(forward_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_DOWN, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(back_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_LEFT, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(left_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_RIGHT, [&](visualization::Visualizer* visualizer) {
			crop_box->Translate(right_move, true);
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_C, [&](visualization::Visualizer* visualizer) {
			*pointcloud_ptr2 = *pointcloud_ptr2->Crop(crop_box->GetOrientedBoundingBox());
			cout << "cropped" << endl;
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_E, [&](visualization::Visualizer* visualizer) {

			*pointcloud_ptr = *CapturePointCloud(&sensor, &intrins.front(), rgbd);
			visualizer->UpdateGeometry();
			cout << "pcl1" << endl;

			visualizer->UpdateRender();
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_S, [&](visualization::Visualizer* visualizer) {

			pointcloud_ptr->RemoveStatisticalOutliers(20, .2);
			if (device_count == 2)
				pointcloud_ptr2->RemoveStatisticalOutliers(20, .2);
			visualizer->UpdateGeometry();
			cout << "removed outlies" << endl;

			visualizer->UpdateRender();
			return false;
		});


		visualizer.RegisterKeyCallback(GLFW_KEY_I, [&](visualization::Visualizer* visualizer) {
			auto camTraj = io::CreatePinholeCameraTrajectoryFromFile("log.log");

			return false;
		});


		visualizer.RegisterKeyCallback(GLFW_KEY_R, [&](visualization::Visualizer* visualizer) {
			pointcloud_ptr2->EstimateNormals();
			pointcloud_ptr2->NormalizeNormals();

			pointcloud_ptr->EstimateNormals();
			pointcloud_ptr->NormalizeNormals();

			cout << "Size before " << pointcloud_ptr->points_.size();
			

			auto down1 = pointcloud_ptr->VoxelDownSample(.025);
			auto down2 = pointcloud_ptr2->VoxelDownSample(.025);

			cout << " Size after "<< pointcloud_ptr->points_.size()<<endl;



			try {
				down1->EstimateNormals();
				down2->EstimateNormals();
				shared_ptr<registration::Feature> feat1 = registration::ComputeFPFHFeature(*down1, geometry::KDTreeSearchParamHybrid(.1, 20));
				shared_ptr<registration::Feature> feat2 = registration::ComputeFPFHFeature(*down2, geometry::KDTreeSearchParamHybrid(.1, 20));

				/*

				registration::RegistrationResult result_ransc = registration::RegistrationRANSACBasedOnFeatureMatching(*down1, *down2, *feat1, *feat2, .02);

				cout << "ransc Fast" << result_ransc.fitness_ << endl;

				pointcloud_ptr->Transform(result_ransc.transformation_);

				visualizer->UpdateGeometry();
				visualizer->UpdateRender();

				Sleep(10000);

				*/
				cout << "Starting fast global" << endl;

				registration::RegistrationResult result_fast = registration::FastGlobalRegistration(*down1, *down2, *feat1, *feat2, registration::FastGlobalRegistrationOption(1.5, true, true, .0, 64, .95, 1000)); //registration::RegistrationICP(*pointcloud_ptr, *pointcloud_ptr2, 1.0);
				std::cout << "Fitness Fast" << result_fast.fitness_ << std::endl;


				cout << "ICP" << endl;




				registration::RegistrationResult result_icp = registration::RegistrationColoredICP(*down1, *down2, .02, result_fast.transformation_, registration::ICPConvergenceCriteria());
				std::cout << "Fitness ransc" << result_icp.fitness_ << std::endl;
				transformMat = result_icp.transformation_;
				cout << "Generated transform mat " << transformMat << endl;
				pointcloud_ptr->Transform(transformMat);

				ofstream trajFile;
				trajFile.open("log.log");
				trajFile << "0    0   1" << endl;
				trajFile << transformMat;
				trajFile.close();

				visualizer->UpdateGeometry();
				visualizer->UpdateRender();

			}
			catch (const std::exception& e) {
				std::cout << "ERRORORORORO    " << e.what() << std::endl;
			}



			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_M, [&](visualization::Visualizer* visualizer) {
			*pointcloud_ptr2 += *pointcloud_ptr;
			pointcloud_ptr->Clear();
			visualizer->UpdateGeometry();
			visualizer->UpdateRender();
			cout << "merged pcl1: " << pointcloud_ptr->points_.size() << " pcl2: " << pointcloud_ptr2->points_.size() << endl;
			return false;
		});

		// Mesh outputting and surface reconstruction
		shared_ptr<TriangleMesh> mesh(new TriangleMesh());
		tuple<shared_ptr<TetraMesh>, vector<size_t>> meshT;
		visualizer.AddGeometry(mesh);
		static int counter = 0;

		visualizer.RegisterKeyCallback(GLFW_KEY_P, [&](visualization::Visualizer* visualizer) {
			counter++;
			io::WritePointCloudToPLY("pcl"+to_string(counter)+".ply", *pointcloud_ptr, false, true);
			return false;
		});

		visualizer.RegisterKeyCallback(GLFW_KEY_O, [&](visualization::Visualizer* visualizer) {
			cout << "Performing Poisson Reconstruction" << endl;
			pointcloud_ptr2->EstimateNormals();
			*mesh = *get<0>(TriangleMesh::CreateFromPointCloudPoisson(*pointcloud_ptr2));
			cout << "Dimensions: " << mesh->Dimension() << endl;
			mesh->ComputeVertexNormals();
			mesh->RemoveDegenerateTriangles();
			cout << "Exporting mesh of all types..." << endl;
			io::WriteTriangleMesh("poisson.gltf", *mesh);

			try {

				io::WritePointCloudToPLY("pcl.ply", *pointcloud_ptr2, false, true);

				pointcloud_ptr->Clear();

				*pointcloud_ptr2 = *pointcloud_ptr2->VoxelDownSample(.01);




				*mesh = *TriangleMesh::CreateFromPointCloudAlphaShape(*pointcloud_ptr2, 3);
				visualizer->UpdateGeometry();
				visualizer->UpdateRender();
				mesh->ComputeVertexNormals();
				mesh->RemoveDegenerateTriangles();
				io::WriteTriangleMesh("alpha3.gltf", *mesh);
				*mesh = *TriangleMesh::CreateFromPointCloudAlphaShape(*pointcloud_ptr2, 1);
				visualizer->UpdateGeometry();
				visualizer->UpdateRender();
				mesh->ComputeVertexNormals();
				mesh->RemoveDegenerateTriangles();
				io::WriteTriangleMesh("alpha1.gltf", *mesh);
				*mesh = *TriangleMesh::CreateFromPointCloudAlphaShape(*pointcloud_ptr2, .5);
				visualizer->UpdateGeometry();
				visualizer->UpdateRender();
				mesh->ComputeVertexNormals();
				mesh->RemoveDegenerateTriangles();
				io::WriteTriangleMesh("alpha0-5.gltf", *mesh);

				*mesh = *TriangleMesh::CreateFromPointCloudAlphaShape(*pointcloud_ptr2, 5);
				visualizer->UpdateGeometry();
				visualizer->UpdateRender();
				mesh->ComputeVertexNormals();
				mesh->RemoveDegenerateTriangles();
				io::WriteTriangleMesh("alpha5.gltf", *mesh);

				vector<double> radi{ 0.05, 0.1, 0.2, 0.4 };
				*mesh = *TriangleMesh::CreateFromPointCloudBallPivoting(*pointcloud_ptr2, radi);
				visualizer->UpdateGeometry();
				visualizer->UpdateRender();
				mesh->ComputeVertexNormals();
				mesh->RemoveDegenerateTriangles();
				io::WriteTriangleMesh("ballPivot.gltf", *mesh);

			}
			catch (exception& e) {
				cout << "ERRRRRRRRRRRRRRRRR          " << e.what() << endl;
			}
			cout << "done" << endl;
			return false;
		});


		do {
			visualizer.PollEvents();
			visualizer.UpdateRender();

			//pointcloud = *pointcloud.CreateFromRGBDImage(*rgbd, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault);
			//PrintPointCloud(pointcloud);
		} while (!flag_exit);


		//geo_added = true;
		/*
		Sleep(3000);



		rgbd = sensor.CaptureFrame(false);
		if (rgbd == nullptr) {
			utility::LogWarning("Null capture!");
			//continue;
			return 1;
		}

		try {
			//geometry::PointCloud::CreateFromRGBDImage(*rgbd, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault, Eigen::Matrix4d::Identity(), true);
			pointcloud_ptr2 = (geometry::PointCloud::CreateFromDepthImage(rgbd->depth_, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault, Eigen::Matrix4d::Identity(), 1000, 3, 1, true));
			pointcloud_ptr2->Rotate(rot, false);
		}
		catch (const std::exception & e) {
			std::cout << "asfdsdf" << e.what() << std::endl;
		}
		pointcloud_ptr2->EstimateNormals();
		pointcloud_ptr2->NormalizeNormals();



		std::cout << "PtCloud 2 captured and displayed" << std::endl;
		//pointcloud_ptr2->Translate(transform);

		visualizer.AddGeometry(pointcloud_ptr2);
		visualizer.UpdateGeometry();
		visualizer.PollEvents();
		visualizer.UpdateRender();


		//Sleep(3000);
		//std::cout << "moved" << std::endl;
		//pc_merge->points_ = pointcloud_ptr->points_;
		//pc_merge->Translate(2 * transform);
		//*pc_merge += *pointcloud_ptr2;
		//pc_merge->Translate(-transform);
		;               //visualizer.AddGeometry(pc_merge);
		visualizer.UpdateGeometry();
		visualizer.PollEvents();
		visualizer.UpdateRender();

		//registration step
		try {
			std::shared_ptr<registration::Feature> feat1 = registration::ComputeFPFHFeature(*pointcloud_ptr, geometry::KDTreeSearchParamHybrid(.01, 150));
			std::shared_ptr<registration::Feature> feat2 = registration::ComputeFPFHFeature(*pointcloud_ptr2, geometry::KDTreeSearchParamHybrid(.01, 150));
			std::cout << "feat1: " << feat1->Dimension() << " feat2: " << feat2->Dimension() << std::endl;


			registration::RegistrationResult result = registration::FastGlobalRegistration(*pointcloud_ptr, *pointcloud_ptr2, *feat1, *feat2, registration::FastGlobalRegistrationOption());
			std::cout << "Fitness from fast registration" << result.fitness_ << std::endl;
			//pointcloud_ptr->Transform(result.transformation_);
			registration::RegistrationResult icpResult = registration::RegistrationICP(*pointcloud_ptr, *pointcloud_ptr2, .02, result.transformation_, registration::TransformationEstimationPointToPoint(), registration::ICPConvergenceCriteria(9.999e-07, 9.999e-07, 1000));
			std::cout << "Fitness from icp registration" << icpResult.fitness_ << std::endl;

			pointcloud_ptr->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
			pointcloud_ptr2->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
			pointcloud_ptr->Transform(icpResult.transformation_);
			visualization::DrawGeometries({ pointcloud_ptr,pointcloud_ptr2 }, "icp result");


			//pointcloud_ptr->Translate(transform);
			//pointcloud_ptr2->Translate(-transform);

			//visualizer.UpdateGeometry();
			visualizer.PollEvents();

			//visualizer.UpdateRender();
		}

		//Sleep(1000);
		//registration::RegistrationResult result2 = registration::FastGlobalRegistration(*pointcloud_ptr, *pointcloud_ptr2, *feat1, *feat2); //registration::RegistrationICP(*pointcloud_ptr, *pointcloud_ptr2, 1.0);
		//std::cout << "Fitness icp" << result2.fitness_ << std::endl;
		//pointcloud_ptr->Transform(result2.transformation_);
		//registration::RegistrationResult result3 = registration::RegistrationRANSACBasedOnFeatureMatching(*pointcloud_ptr, *pointcloud_ptr2,*feat1,*feat2,1.0);
		//std::cout << "Fitness ransc" << result3.fitness_ << std::endl;


		catch (const std::exception & e) {
			std::cout << "ERRORORORORO    " << e.what() << std::endl;
		}


		//visualizer.RemoveGeometry(pc_merge);
		//visualizer.UpdateGeometry();
		visualizer.PollEvents();

		visualizer.UpdateRender();




		do {
			visualizer.PollEvents();
			visualizer.UpdateRender();

			//pointcloud = *pointcloud.CreateFromRGBDImage(*rgbd, camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault);
			//PrintPointCloud(pointcloud);
		} while (!flag_exit);
		//*pointcloud_ptr = pointcloud;
		//pointcloud_ptr->NormalizeNormals();
		//auto bounding_box = pointcloud_ptr->GetAxisAlignedBoundingBox();
		*/

	}
	else {
		utility::LogWarning("Unrecognized option: {}\n", option);
		return 1;
	}

	utility::LogInfo("End of the test.\n");

	return 0;

}

