#include <opencv2/opencv.hpp>

cv::RNG rng(12345);

//@url: http://answers.opencv.org/question/75649/object-detection-and-splitting-clustering/
typedef struct contour_t {
  std::vector<cv::Point> contour_pts;
  int idx;
  int unique_id;
  cv::Rect bounding_box;

  contour_t() :
      contour_pts(), idx(-1), unique_id(-1), bounding_box() {
  }

  contour_t(const contour_t &copy) {
    contour_pts = copy.contour_pts;
    idx = copy.idx;
    unique_id = copy.unique_id;
    bounding_box = copy.bounding_box;
  }

  contour_t(const std::vector<cv::Point> &c, const int unique, const cv::Rect &bb) :
      contour_pts(c), idx(-1), unique_id(unique), bounding_box(bb) {
  }
} contour_t;

typedef struct contours_info_t {
  std::vector<contour_t> contours_list;
  std::vector<size_t> index_contour_points_list;
  size_t nb_contour_points;
} contours_info_t;

double pointDistance(const cv::Point &pt1, const cv::Point &pt2) {
  return sqrt((double) ((pt1.x - pt2.x) * (pt1.x - pt2.x)  + (pt1.y - pt2.y) * (pt1.y - pt2.y)));
}

double contourDistance(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2) {
  double min_dist = std::numeric_limits<double>::max();

  for (std::vector<cv::Point>::const_iterator it1 = contour1.begin(); it1 != contour1.end(); ++it1) {
    for (std::vector<cv::Point>::const_iterator it2 = contour2.begin(); it2 != contour2.end(); ++it2) {
      double dist = pointDistance(*it1, *it2);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
  }

  return min_dist;
}

bool sameContour(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2) {
  if (contour1.size() != contour2.size()) {
    return false;
  }

  for (size_t i = 0; i < contour1.size(); i++) {
    if (contour1[i].x != contour2[i].x || contour1[i].y != contour2[i].y) {
      return false;
    }
  }

  return true;
}

double boundingBoxDistance(const cv::Rect &bb1, const cv::Rect &bb2) {
  double min_dist = std::numeric_limits<double>::max();

  //Top left
  min_dist = std::min(pointDistance(bb1.tl(), bb2.tl()), pointDistance(bb1.tl(), bb2.tl() + cv::Point(bb2.width, 0)));
  min_dist = std::min(min_dist, pointDistance(bb1.tl(), bb2.br()));
  min_dist = std::min(min_dist, pointDistance(bb1.tl(), bb2.br() - cv::Point(bb2.width, 0)));

  //Top right
  min_dist = std::min(min_dist, pointDistance(bb1.tl() + cv::Point(bb1.width, 0), bb2.tl()));
  min_dist = std::min(min_dist, pointDistance(bb1.tl() + cv::Point(bb1.width, 0), bb2.tl() + cv::Point(bb2.width, 0)));
  min_dist = std::min(min_dist, pointDistance(bb1.tl() + cv::Point(bb1.width, 0), bb2.br()));
  min_dist = std::min(min_dist, pointDistance(bb1.tl() + cv::Point(bb1.width, 0), bb2.br() - cv::Point(bb2.width, 0)));

  //Bottom right
  min_dist = std::min(min_dist, pointDistance(bb1.br(), bb2.tl()));
  min_dist = std::min(min_dist, pointDistance(bb1.br(), bb2.tl() + cv::Point(bb2.width, 0)));
  min_dist = std::min(min_dist, pointDistance(bb1.br(), bb2.br()));
  min_dist = std::min(min_dist, pointDistance(bb1.br(), bb2.br() - cv::Point(bb2.width, 0)));

  //Bottom left
  min_dist = std::min(min_dist, pointDistance(bb1.br() - cv::Point(bb1.width, 0), bb2.tl()));
  min_dist = std::min(min_dist, pointDistance(bb1.br() - cv::Point(bb1.width, 0), bb2.tl() + cv::Point(bb2.width, 0)));
  min_dist = std::min(min_dist, pointDistance(bb1.br() - cv::Point(bb1.width, 0), bb2.br()));
  min_dist = std::min(min_dist, pointDistance(bb1.br() - cv::Point(bb1.width, 0), bb2.br() - cv::Point(bb2.width, 0)));

  return min_dist;
}

std::vector<contour_t> findNeighborContours(const contours_info_t &contours_info, const contour_t &contour,
    const double min_cluster_dist, cv::flann::Index &kdTree, const bool fast=true, const bool useKdTree=true) {
  std::vector<contour_t> neighbors;

  if (useKdTree) {
    int nb_points = contours_info.contours_list.size()*4;
    cv::Mat query_pt(1, 2, CV_32F);
    std::vector<int> already_processed;

    if (fast) {
      cv::Mat indices(1, nb_points, CV_32S);
      cv::Mat dists(1, nb_points, CV_32F);

      //Top left
      query_pt.at<float>(0,0) = contour.bounding_box.tl().x;
      query_pt.at<float>(0,1) = contour.bounding_box.tl().y;
      int nb_res = kdTree.radiusSearch(query_pt, indices, dists, min_cluster_dist, nb_points);
      for (int i = 0; i < nb_res; i++) {
        int index = indices.at<int>(0,i) / 4;
        if (std::find(already_processed.begin(), already_processed.end(), index) == already_processed.end()) {
          if (contours_info.contours_list[index].unique_id != contour.unique_id) {
            already_processed.push_back(index);
            neighbors.push_back(contours_info.contours_list[index]);
          }
        }
      }

      //Top right
      query_pt.at<float>(0,0) = contour.bounding_box.tl().x + contour.bounding_box.width;
      query_pt.at<float>(0,1) = contour.bounding_box.tl().y;
      nb_res = kdTree.radiusSearch(query_pt, indices, dists, min_cluster_dist, nb_points);
      for (int i = 0; i < nb_res; i++) {
        int index = indices.at<int>(0,i) / 4;
        if (std::find(already_processed.begin(), already_processed.end(), index) == already_processed.end()) {
          if (contours_info.contours_list[index].unique_id != contour.unique_id) {
            already_processed.push_back(index);
            neighbors.push_back(contours_info.contours_list[index]);
          }
        }
      }

      //Bottom right
      query_pt.at<float>(0,0) = contour.bounding_box.br().x;
      query_pt.at<float>(0,1) = contour.bounding_box.br().y;
      nb_res = kdTree.radiusSearch(query_pt, indices, dists, min_cluster_dist, nb_points);
      for (int i = 0; i < nb_res; i++) {
        int index = indices.at<int>(0,i) / 4;
        if (std::find(already_processed.begin(), already_processed.end(), index) == already_processed.end()) {
          if (contours_info.contours_list[index].unique_id != contour.unique_id) {
            already_processed.push_back(index);
            neighbors.push_back(contours_info.contours_list[index]);
          }
        }
      }

      //Bottom left
      query_pt.at<float>(0,0) = contour.bounding_box.br().x - contour.bounding_box.width;
      query_pt.at<float>(0,1) = contour.bounding_box.br().y;
      nb_res = kdTree.radiusSearch(query_pt, indices, dists, min_cluster_dist, nb_points);
      for (int i = 0; i < nb_res; i++) {
        int index = indices.at<int>(0,i) / 4;
        if (std::find(already_processed.begin(), already_processed.end(), index) == already_processed.end()) {
          if (contours_info.contours_list[index].unique_id != contour.unique_id) {
            already_processed.push_back(index);
            neighbors.push_back(contours_info.contours_list[index]);
          }
        }
      }
    } else {
      nb_points = contours_info.nb_contour_points;
      std::vector<size_t> vector_raw_index = contours_info.index_contour_points_list;

      cv::Mat indices(1, nb_points, CV_32S);
      cv::Mat dists(1, nb_points, CV_32F);

      for (std::vector<cv::Point>::const_iterator it = contour.contour_pts.begin(); it != contour.contour_pts.end(); ++it) {
        query_pt.at<float>(0,0) = it->x;
        query_pt.at<float>(0,1) = it->y;

        int nb_res = kdTree.radiusSearch(query_pt, indices, dists, min_cluster_dist, nb_points);
        for (int i = 0; i < nb_res; i++) {
          int raw_index = indices.at<int>(0,i);

          if (std::find(already_processed.begin(), already_processed.end(), vector_raw_index[raw_index]) ==
              already_processed.end()) {
            if (contours_info.contours_list[vector_raw_index[raw_index]].unique_id != contour.unique_id) {
              already_processed.push_back(vector_raw_index[raw_index]);
              neighbors.push_back(contours_info.contours_list[vector_raw_index[raw_index]]);
            }
          }
        }
      }
    }
  } else {
    for (std::vector<contour_t>::const_iterator it = contours_info.contours_list.begin();
        it != contours_info.contours_list.end(); ++it) {
      if (!sameContour(it->contour_pts, contour.contour_pts)) {
        if(fast) {
          if (boundingBoxDistance(it->bounding_box, contour.bounding_box) <= min_cluster_dist) {
            neighbors.push_back(*it);
          }
        } else {
          if (contourDistance(it->contour_pts, contour.contour_pts) <= min_cluster_dist) {
            neighbors.push_back(*it);
          }
        }
      }
    }
  }

  return neighbors;
}

std::map<int, std::vector<std::vector<cv::Point> > > clustering(const std::vector<std::vector<cv::Point> > &contours,
    const double min_cluster_dist, const int minNeighbors=2, const bool fast=true, const bool useKdTree=true) {
  int idx = 0;
  std::map<int, std::vector<std::vector<cv::Point> > > clustered_contours;
  std::vector<contour_t> queue;
  std::vector<contour_t> contour_labels(contours.size());
  std::vector<size_t> contour_points_index;
  size_t nb_rows = 0;
  for (size_t i = 0; i < contours.size(); i++) {
    cv::Rect bb = cv::boundingRect(contours[i]);
    contour_labels[i] = contour_t(contours[i], (int) i, bb);
    nb_rows += contours[i].size();

    for (size_t j = 0; j < contours[i].size(); j++) {
      contour_points_index.push_back(i);
    }
  }

  //KDTree
  cv::flann::KDTreeIndexParams kdtree_index;
  cv::Mat features;
  cv::flann::Index kdTree;
  if (useKdTree) {
    if(fast) {
      features = cv::Mat(contours.size()*4, 2, CV_32F);

      for (size_t i = 0; i < contour_labels.size(); i++) {
        //Top left
        features.at<float>(i*4, 0) = contour_labels[i].bounding_box.tl().x;
        features.at<float>(i*4, 1) = contour_labels[i].bounding_box.tl().y;

        //Top right
        features.at<float>(i*4 + 1, 0) = contour_labels[i].bounding_box.tl().x + contour_labels[i].bounding_box.width;
        features.at<float>(i*4 + 1, 1) = contour_labels[i].bounding_box.tl().y;

        //Bottom right
        features.at<float>(i*4 + 2, 0) = contour_labels[i].bounding_box.br().x;
        features.at<float>(i*4 + 2, 1) = contour_labels[i].bounding_box.br().y;

        //Bottom left
        features.at<float>(i*4 + 3, 0) = contour_labels[i].bounding_box.br().x - contour_labels[i].bounding_box.width;
        features.at<float>(i*4 + 3, 1) = contour_labels[i].bounding_box.br().y;
      }
    } else {
      features = cv::Mat(nb_rows, 2, CV_32F);
      int row_index = 0;
      for (size_t i = 0; i < contour_labels.size(); i++) {
        for (size_t j = 0; j < contour_labels[i].contour_pts.size(); j++) {
          features.at<float>(row_index, 0) = contour_labels[i].contour_pts[j].x;
          features.at<float>(row_index, 1) = contour_labels[i].contour_pts[j].y;
          row_index++;
        }
      }
    }

    //With OpenCV 2.4.9, it seems that FLANN_DIST_L1 is swapped with FLANN_DIST_L2 / cvflann::EUCLIDEAN ?
    kdTree.build(features, kdtree_index, cvflann::FLANN_DIST_L1);
  }

  contours_info_t contours_info;
  contours_info.contours_list = contour_labels;
  contours_info.nb_contour_points = nb_rows;
  contours_info.index_contour_points_list = contour_points_index;

  std::vector<int> already_processed;
  for (size_t i = 0; i < contour_labels.size(); i++) {
    if (std::find(already_processed.begin(), already_processed.end(),
        contour_labels[i].unique_id) != already_processed.end()) {
      continue;
    }

    queue.push_back(contour_labels[i]);
    already_processed.push_back(contour_labels[i].unique_id);

    for (size_t j = 0; j < queue.size(); j++) {
      std::vector<contour_t> neighbor_contours = findNeighborContours(contours_info, queue[j], min_cluster_dist,
          kdTree, fast, useKdTree);

      if (neighbor_contours.size() >= minNeighbors) {
        for (size_t k = 0; k < neighbor_contours.size(); k++) {
          if (std::find(already_processed.begin(), already_processed.end(),
              neighbor_contours[k].unique_id) == already_processed.end()) {
            queue.push_back(neighbor_contours[k]);
            already_processed.push_back(neighbor_contours[k].unique_id);
          }
        }
      }
    }

    if (queue.size() >= minNeighbors) {
      for (std::vector<contour_t>::iterator it = queue.begin(); it != queue.end(); ++it) {
        it->idx = idx;
        clustered_contours[idx].push_back(it->contour_pts);
      }
    }

    idx++;
    queue.clear();
  }

  return clustered_contours;
}

bool compareRectInv(const cv::Rect &r1, const cv::Rect &r2) {
  return (r1.area() > r2.area());
}

std::vector<cv::Rect> nonMaxSuppression(const std::map<int, std::vector<std::vector<cv::Point> > > &clusterised_contours,
    const double overlap=0.8) {
  std::vector<cv::Rect> vectorOfBoundingBoxes;

  for (std::map<int, std::vector<std::vector<cv::Point> > >::const_iterator it1 = clusterised_contours.begin();
      it1 != clusterised_contours.end(); ++it1) {
    std::vector<cv::Point> cluster;

    for (std::vector<std::vector<cv::Point> >::const_iterator it2 = it1->second.begin();
        it2 != it1->second.end(); it2++) {
      for (std::vector<cv::Point>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++) {
        cluster.push_back(*it3);
      }
    }

    cv::Rect boundingBox = cv::boundingRect(cluster);
    vectorOfBoundingBoxes.push_back(boundingBox);
  }

  std::sort(vectorOfBoundingBoxes.begin(), vectorOfBoundingBoxes.end(), compareRectInv);
  for (size_t cpt = 0; cpt < vectorOfBoundingBoxes.size(); cpt++) {
    for (std::vector<cv::Rect>::iterator it2 = vectorOfBoundingBoxes.begin()+cpt+1; it2 != vectorOfBoundingBoxes.end(); ) {
      cv::Rect r1 = vectorOfBoundingBoxes[cpt];
      cv::Rect r2 = *it2;
      cv::Rect rOverlap = r1 & r2;

      if(rOverlap.area() > overlap*r2.area()) {
        it2 = vectorOfBoundingBoxes.erase(it2);
      } else {
        it2++;
      }
    }
  }

  return vectorOfBoundingBoxes;
}

void contourDetection(const cv::Mat &img, const cv::Mat &img_contour, const double min_contour_dist,
    const int minNeighbors=2, const bool fast=true, const bool useKdTree=true) {
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(img_contour, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat img_contour_box;
  img.copyTo(img_contour_box);
  for (std::vector<std::vector<cv::Point> >::const_iterator it = contours.begin(); it != contours.end(); ++it) {
    cv::Rect bb = cv::boundingRect(*it);
    cv::rectangle(img_contour_box, bb, cv::Scalar(255,0,0), 1);
  }
  cv::imshow("Contours bounding box", img_contour_box);

  double t = (double) cv::getTickCount();
  std::map<int, std::vector<std::vector<cv::Point> > > clusterised_contours = clustering(contours, min_contour_dist,
      minNeighbors, fast, useKdTree);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << "clusterised_contours=" << clusterised_contours.size() << std::endl;
  std::cout << "time=" << t << " s" << std::endl;

  cv::Mat img2;
  img.copyTo(img2);

//  for (std::map<int, std::vector<std::vector<cv::Point> > >::const_iterator it1 =
//      clusterised_contours.begin(); it1 != clusterised_contours.end(); ++it1) {
//    std::vector<cv::Point> cluster;
//    cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//    for (std::vector<std::vector<cv::Point> >::const_iterator it2 = it1->second.begin();
//        it2 != it1->second.end(); it2++) {
//      for (std::vector<cv::Point>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++) {
//        cluster.push_back(*it3);
//      }
//    }
//
////    cv::RotatedRect rotatedRect = cv::minAreaRect(cluster);
////    cv::Point2f vertices[4];
////    rotatedRect.points(vertices);
////    for (int i = 0; i < 4; i++) {
////      cv::line(img2, vertices[i], vertices[(i + 1) % 4], color, 3);
////    }
//
//    cv::Rect boundingBox = cv::boundingRect(cluster);
//    cv::rectangle(img2, boundingBox, color, 2);
//  }

  std::vector<cv::Rect> vectorOfBoundingBoxes = nonMaxSuppression(clusterised_contours);
  for (std::vector<cv::Rect>::const_iterator it = vectorOfBoundingBoxes.begin(); it != vectorOfBoundingBoxes.end(); ++it) {
    cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::rectangle(img2, *it, color, 2);
  }

  cv::imshow("img2", img2);
  cv::imwrite("image_clustering.png", img2);
}

void binarise(const cv::Mat &img, cv::Mat &img_binarise, const double thresh, const int type) {
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  cv::threshold(img_gray, img_binarise, thresh, 255, type);
}

void cannySimple(const cv::Mat &img, cv::Mat &img_contour, const double thresh1, const double thresh2, const cv::Size &kernel) {
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  if (kernel.area() > 0) {
    cv::blur(img_gray, img_gray, kernel);
  }

  cv::Canny(img_gray, img_contour, thresh1, thresh2, 3, true);
}

void cannyOtsu(const cv::Mat &img, cv::Mat &img_contour, const cv::Size &kernel) {
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  double otsu = cv::threshold(img_gray, img_gray.clone(), 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  std::cout << "otsu=" << otsu << std::endl;

  cv::Canny(img_gray, img_contour, otsu*0.5, otsu, 3, true);
}

void test1() {
  cv::Mat img = cv::imread("zB7lw.jpg");

  cv::imshow("Image", img);

  cv::Mat img_contour;
  const double canny_threshold = 12;
  cannySimple(img, img_contour, canny_threshold, canny_threshold*3, cv::Size(3,3));

  cv::imshow("Canny", img_contour);

  const double min_contour_dist = 15.0;
  const int minNeighbors = 2;
  const bool fast = true, useKdTree = true;
  contourDetection(img, img_contour, min_contour_dist, minNeighbors, fast, useKdTree);
}

void test2() {
  cv::Mat img = cv::imread("scanned_receipt_5.jpg");

  cv::imshow("Image", img);

  cv::Mat img_contour;
  const double canny_threshold = 12;
  cannySimple(img, img_contour, canny_threshold, canny_threshold*3, cv::Size(3,3));

  cv::imshow("Canny", img_contour);

  const double min_contour_dist = 3.0;
  const int minNeighbors = 2;
  const bool fast = true, useKdTree = true;
  contourDetection(img, img_contour, min_contour_dist, minNeighbors, fast, useKdTree);
}

void test3() {
  cv::Mat img = cv::imread("scanned_receipt_5_3.png");

  cv::imshow("Image", img);

  cv::Mat img_contour;
  const double canny_threshold = 12;
  cannySimple(img, img_contour, canny_threshold, canny_threshold*3, cv::Size(3,3));

  cv::imshow("Canny", img_contour);

  const double min_contour_dist = 30.0;
  const int minNeighbors = 2;
  const bool fast = false, useKdTree = true;
  contourDetection(img, img_contour, min_contour_dist, minNeighbors, fast, useKdTree);
}

void test4() {
  cv::Mat img = cv::imread("scanned_receipt_5.jpg");

  cv::imshow("Image", img);

  cv::Mat img_binarise;
  const double threshold = 200;
  binarise(img, img_binarise, threshold, cv::THRESH_BINARY_INV);

  cv::Mat img_horizontal_projection;
  img.copyTo(img_horizontal_projection);
  for (int i = 0; i < img_binarise.cols; i++) {
      int nonzero = cv::countNonZero(img_binarise.col(i));
      cv::line(img_horizontal_projection, cv::Point(i,0), cv::Point(i,nonzero/2), cv::Scalar(0),1);
  }

  std::vector<int> zero_locations;
  cv::reduce(img_binarise, zero_locations, 0, CV_REDUCE_SUM);
  for (std::vector<int>::const_iterator it = zero_locations.begin(); it != zero_locations.end(); ++it) {
    std::cout << (*it) << std::endl;
  }

  cv::imshow("Binarise", img_binarise);
  cv::imshow("img_horizontal_projection", img_horizontal_projection);

  const double min_contour_dist = 15.0;
  const int minNeighbors = 2;
  const bool fast = false, useKdTree = true;
  contourDetection(img, img_binarise, min_contour_dist, minNeighbors, fast, useKdTree);
}

void test5() {
  cv::Mat img = cv::imread("scanned_receipt_5_2.png");

  cv::imshow("Image", img);

  cv::Mat img_binarise;
  const double threshold = 200;
  binarise(img, img_binarise, threshold, cv::THRESH_BINARY_INV);

  cv::Mat img_horizontal_projection;
  img.copyTo(img_horizontal_projection);
  for (int i = 0; i < img_binarise.cols; i++) {
      int nonzero = cv::countNonZero(img_binarise.col(i));
      cv::line(img_horizontal_projection, cv::Point(i,0), cv::Point(i,nonzero/2), cv::Scalar(0),1);
  }

  std::vector<int> zero_locations;
  cv::reduce(img_binarise, zero_locations, 0, CV_REDUCE_SUM);
  int start = 0, end = 0;
  bool isZero = true;
  for (size_t cpt = 0; cpt < zero_locations.size(); cpt++) {
    if (!isZero) {
      if (zero_locations[cpt] == 0) {
        isZero = true;
        start = cpt;
      }
    } else {
      if (zero_locations[cpt] != 0) {
        isZero = false;
        end = cpt;

        int meanLocation = (end - start) / 2.0 + start;
        if(end - start > 5) {
          cv::line(img_horizontal_projection, cv::Point(meanLocation, 0),
              cv::Point(meanLocation, img_horizontal_projection.rows-1), cv::Scalar(0,0,255), 3);
        }
      }
    }
  }

  if (isZero) {
    end = zero_locations.size() -1;
    int meanLocation = (end - start) / 2.0 + start;
    if(end - start > 5) {
      cv::line(img_horizontal_projection, cv::Point(meanLocation, 0),
          cv::Point(meanLocation, img_horizontal_projection.rows-1), cv::Scalar(0,0,255), 3);
    }
  }

  cv::imshow("Binarise", img_binarise);
  cv::imshow("img_horizontal_projection", img_horizontal_projection);
  cv::imwrite("img_horizontal_projection.png", img_horizontal_projection);

  const double min_contour_dist = 15.0;
  const int minNeighbors = 2;
  const bool fast = true, useKdTree = true;
  contourDetection(img, img_binarise, min_contour_dist, minNeighbors, fast, useKdTree);
}

int main() {
//  test1();
//  test2();
//  test3();
//  test4();
//  test5();

  cv::waitKey(0);
  return 0;
}