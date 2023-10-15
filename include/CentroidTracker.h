#include <iostream>
#include <string>

#include <ros/ros.h>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <math.h>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
#include <boost/range/combine.hpp>
#include <opencv2/opencv.hpp>

namespace cent_ns { 

typedef std::pair<double, double> centroid_t;
typedef std::unordered_map< int /* Centroid ID */, centroid_t > centroidMap_t;
typedef std::vector< std::vector< double > > distMatrix_t;
constexpr int INVALID_ID = -1;  

template<typename T>
std::vector<std::size_t> argsort(const std::vector<T> &v) {
  std::vector<std::size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  stable_sort(idx.begin(), idx.end(),
       [&v](std::size_t i1, std::size_t i2) {return v[i1] < v[i2];});
  return idx;
}

template<class T>
std::ostream& operator<<(std::ostream& stream, const std::pair<T, T>& pair) {
    stream << " (" << pair.first << ", " << pair.second << ") ";
    return stream; 
}

template<class T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& values) {
  stream << "[ ";
  for (const auto& element : values) {
    stream << element << " ";
  }
  stream << " ]";
  return stream;
}

class CentroidTracker {

public:
CentroidTracker(int maxFrames) : 
  m_maxDisappearedFrames (maxFrames) { }

const int updateAllCentroids(const std::vector<cent_ns::centroid_t>& t_inputCentroids) {  
  ROS_INFO("CentroidTracker - update called");
  ROS_DEBUG_STREAM("Input centroids: " << t_inputCentroids);
  ROS_DEBUG_STREAM_COND(!m_centroidMap.empty(), "Current centroids: ");
  for (auto& centroid : m_centroidMap) ROS_DEBUG_STREAM(centroid.second);

  if (t_inputCentroids.empty()) {
    // No new centroids need updating
    case_handle_empty_centroid_input();
    ROS_WARN("CentroidTracker - input vector is empty, returning -1");
    return -1;
  }
  else if (m_centroidMap.empty()) {
    // If there are no objects stored, store all new centroids
    ROS_WARN("CentroidTracker - no objects currently stored");
    for (const auto& newCentroid : t_inputCentroids) {
        register_centroid(newCentroid);
    }
  } else {
    case_match_input_to_centroids(t_inputCentroids);
  }
  
  update_currently_tracked_ID();
  if (m_currentlyTrackedID == -1) {
    return -1;
  }
  
  auto it = std::find(t_inputCentroids.begin(), t_inputCentroids.end(), m_centroidMap[m_currentlyTrackedID]);
  if (it != t_inputCentroids.end()) {
    return int(it - t_inputCentroids.begin());
  }

  // If the currently tracked centroid is not in the inputCentroids vector
  for (const auto& mapItem : m_centroidMap) {
      auto newIt = std::find(t_inputCentroids.begin(), t_inputCentroids.end(), mapItem.second);
      if (newIt != t_inputCentroids.end()) {
          m_currentlyTrackedID = int(newIt - t_inputCentroids.begin());
          return m_currentlyTrackedID;
      }
  }

  ROS_FATAL("CentroidTracker - Something went wrong. WTF?!");
  return -1; 
}   

void drawCentroids(cv::Mat& image) {
  for(const auto& centroid : m_centroidMap) {
    cv::putText(image, 
      std::to_string(centroid.first), 
      cv::Point(centroid.second.first, centroid.second.second),
      cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,120,215), 2, cv::LINE_AA);
  }
}

private:
void update_currently_tracked_ID() {
  if (m_centroidMap.empty()) {
    m_currentlyTrackedID = -1;
    return;
  }

  if (m_currentlyTrackedID == -1 ||
      m_centroidMap.find(m_currentlyTrackedID) == m_centroidMap.end()) {
    m_currentlyTrackedID = m_centroidMap.begin()->first;
  }
}

void case_match_input_to_centroids(const std::vector<cent_ns::centroid_t>& t_inputCentroids) {
  ROS_INFO("CantroidTracker::case_match_input_to_centroids");
  std::vector<int /* IDs */> centroidIDs;
  std::vector<cent_ns::centroid_t> centroidValues;
  for (auto& centroid : m_centroidMap) {
    centroidIDs.push_back(centroid.first);
    centroidValues.push_back(centroid.second);
  }

  // Compute distances between all pairs of existing and input centroids
  ROS_DEBUG("D: ");
  int D_rowCount = centroidValues.size(), D_colCount = t_inputCentroids.size();
  distMatrix_t D (D_rowCount, std::vector<double>(D_colCount, 0.0));
  for (std::size_t i = 0; i < centroidValues.size(); i++) {
    for (std::size_t j = 0; j < t_inputCentroids.size(); j++) {
      D[i][j] = distance(centroidValues[i], t_inputCentroids[j]);
    }
    ROS_DEBUG_STREAM(D[i]);
  }
  
  // Do argsort on all rows
  std::vector<double> closestDistanceInRow;
  for (const auto& distanceRow : D) {
    closestDistanceInRow.push_back(
      *std::min_element(distanceRow.begin(), distanceRow.end()));
  }
  // Sorted current centroid indices wrt. the distance to the closes input centroid
  // e.g. the first element in this vector will have a closest input centroid
  std::vector<std::size_t> rows = argsort(closestDistanceInRow);
  ROS_DEBUG_STREAM("Min of each row: " << closestDistanceInRow); 
  ROS_DEBUG_STREAM("^argsorted: " << rows);

  // Calculated list of indices of minimum values in distance matrix
  std::vector<std::size_t> minRowDistanceIndices;
  for (const auto& distanceRow : D) {
    minRowDistanceIndices.push_back(
      std::size_t(std::min_element(distanceRow.begin(), distanceRow.end()) - distanceRow.begin())
    );
  }
  ROS_DEBUG_STREAM("Argmin of each row: " << minRowDistanceIndices);

  // Sort minimum distance indices in each row according to the sorted distance indices vector
  std::vector<std::size_t> cols;
  for(const auto& index : rows) {
    cols.push_back(minRowDistanceIndices[index]);
  }
  ROS_DEBUG_STREAM("Sorted wrt. the argsort(min of each row): " << cols);

  std::set<std::size_t /* index */> usedRowIndices, usedColumnIndices;
  for (auto item : boost::combine(rows, cols)) {
    std::size_t row, col;
    boost::tie(row, col) = item;  

    if (usedRowIndices.find(row) != usedRowIndices.end()
      || usedColumnIndices.find(col) != usedColumnIndices.end()) {
        continue;
    }

    int centroidID = centroidIDs[row];
    ROS_WARN_STREAM(m_centroidMap[centroidID] << " -> " << t_inputCentroids[col]);
    m_centroidMap[centroidID] = t_inputCentroids[col];
    m_disappearedCentroidMap[centroidID] = 0;

    usedRowIndices.insert(row);
    usedColumnIndices.insert(col);
  }

  std::vector<std::size_t> rowIDs(D_rowCount), colIDs(D_colCount), unusedRowIDs, unusedColIDs;
  std::iota(rowIDs.begin(), rowIDs.end(), 0); 
  std::iota(colIDs.begin(), colIDs.end(), 0);
  std::set_difference(rowIDs.begin(), rowIDs.end(),
    usedRowIndices.begin(), usedRowIndices.end(),
    std::inserter(unusedRowIDs, unusedRowIDs.begin()));
  std::set_difference(colIDs.begin(), colIDs.end(),
    usedColumnIndices.begin(), usedColumnIndices.end(), 
    std::inserter(unusedColIDs, unusedColIDs.begin()));
  
  // if there is more inputs than centroids
  if (D_rowCount < D_colCount) {
    for (const auto& col : unusedColIDs) {
      register_centroid(t_inputCentroids[col]);
    }
  }

  // Go through all the unused IDs and remove increase the dissapearance map
  for (const auto& row : unusedRowIDs) {
    std::size_t centroidID = centroidIDs[row];
    m_disappearedCentroidMap[centroidID]++;
    ROS_WARN("Centroid ID: %lu - (%.2f, %.2f) unseen for %d frames", centroidID, 
        m_centroidMap[centroidID].first, 
        m_centroidMap[centroidID].second, 
        m_disappearedCentroidMap[centroidID]);

    if (m_disappearedCentroidMap[centroidID] > m_maxDisappearedFrames) {
        deregister_centroid_id(centroidID);
    }
  }
}

void case_handle_empty_centroid_input() {
  ROS_WARN("CantroidTracker::case_handle_empty_centroid_input");
  // Add all the objects to disappearance map, if they're not there
  for (auto& objCentroid : m_centroidMap){
    m_disappearedCentroidMap.emplace(objCentroid.first, 0);
  } 

  // Make objects disappear
  std::vector<std::size_t> IDsToRemove;
  for (auto& centroid : m_disappearedCentroidMap) {
    centroid.second++;
    ROS_WARN("Object: (%.2f, %.2f) missing for %d frames",
      m_centroidMap[centroid.first].first, 
      m_centroidMap[centroid.first].second,
      centroid.second);
    if (centroid.second > m_maxDisappearedFrames) {
      IDsToRemove.push_back(centroid.first);
    }
  }

  for (const auto& id : IDsToRemove) {
    deregister_centroid_id(id);
  }
}

static const cent_ns::centroid_t get_closest_centroid(
  const cent_ns::centroid_t& centroid, const std::vector<cent_ns::centroid_t>& centroids) {
  double minDist = 1e10;
  std::size_t closestIndex = -1;
  for (std::size_t i = 0; i < centroids.size(); i++) {
    double dist = distance(centroid, centroids.at(i));
    if (dist < minDist) {
      closestIndex = i;
      minDist = dist; 
    }
  }
  return centroids.at(closestIndex);
}

static inline double distance(const cent_ns::centroid_t& c1, const cent_ns::centroid_t& c2) {
  return sqrt(pow(c1.first - c2.first , 2) + pow(c1.second - c2.second , 2));
}

void register_centroid(const cent_ns::centroid_t& newCentroid) {
  m_centroidMap[m_nextObjectID] = newCentroid;
  m_disappearedCentroidMap[m_nextObjectID] = 0;
  ROS_INFO("New object added! - (%.2f, %.2f)", newCentroid.first, newCentroid.second);
  m_nextObjectID++;
}

void deregister_centroid_id(const int objectID) {
  ROS_FATAL("Removing object with ID: %d - (%.2f, %.2f)", 
    objectID, 
    m_centroidMap[objectID].first, 
    m_centroidMap[objectID].second);
  m_centroidMap.erase(objectID);
  m_disappearedCentroidMap.erase(objectID);
}

int m_nextObjectID = 0;
int m_currentlyTrackedID = -1;
const int m_maxDisappearedFrames;
cent_ns::centroidMap_t m_centroidMap;
std::unordered_map< 
  int /* Centroid ID */, 
  int /* Disappeared frame count */ > m_disappearedCentroidMap;
};
}