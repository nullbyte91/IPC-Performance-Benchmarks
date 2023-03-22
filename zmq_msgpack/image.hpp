#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <chrono>
#include <ctime>
#include <iostream>
#include <msgpack.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <zmq.hpp>
#include <zmq_addon.hpp>

struct Image {
  std::vector<uchar> matrix;
  int rows = 0;
  int cols = 0;
  int type = 0;
  MSGPACK_DEFINE(matrix, rows, cols, type);
};

#endif  //IMAGE_HPP