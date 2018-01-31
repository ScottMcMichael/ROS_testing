/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include "tf2/transform_storage.h"

#include <deque>
#include <queue>

#include <sstream>

#include <ros/message_forward.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(TransformStamped);
}

namespace tf2_mod
{
using tf2::CompactFrameID;
using tf2::TransformStorage;

typedef std::pair<ros::Time, CompactFrameID> P_TimeAndFrameID;
/*
class TimeCacheInterface
{
public:
  /// Access data from the cache
  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0)=0; //returns false if data unavailable (should be thrown as lookup exception

  /// Insert data into the cache
  virtual bool insertData(const TransformStorage& new_data)=0;

  /// Clear the list of stored values
  virtual void clearList()=0;

  /// Retrieve the parent at a specific time
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str) = 0;

  /// Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
  virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;


  // Debugging information methods
  /// Get the length of the stored list
  virtual unsigned int getListLength()=0;

  /// Get the latest timestamp cached
  virtual ros::Time getLatestTimestamp()=0;

  /// Get the oldest timestamp cached
  virtual ros::Time getOldestTimestamp()=0;
};
typedef boost::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;
*/

/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. *//*
class TimeCache : public TimeCacheInterface
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache(ros::Duration  max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));


  /// Virtual methods

  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0);
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual ros::Time getLatestTimestamp();
  virtual ros::Time getOldestTimestamp();
  

private:
  typedef std::deque<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  ros::Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two, ros::Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);


  void pruneList();



};

class StaticCache : public TimeCacheInterface
{
 public:
  /// Virtual methods

  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0); //returns false if data unavailable (should be thrown as lookup exception
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();


  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual ros::Time getLatestTimestamp();
  virtual ros::Time getOldestTimestamp();
  

private:
  
  TransformStorage  storage_;
};
*/

/** \brief Similar to TimeCache except that the number of entries is kept under a 
 * specified count using a FIFO method.
 *//*
class CountCache : public TimeCache
{
 public:

  CountCache(size_t max_entries = 10000); // TODO: Adjust the default length

  // The public interface is the same as the TimeCache class.

  virtual bool insertData(const TransformStorage& new_data);


private:

  /// Record the data timestampss in the order they are inserted.
  std::queue<ros::Time> insertion_order_queue_;
  
  // Override the prune behavior from TimeCache
  void pruneList();

}; // End class CountCache
*/
// TODO: Don't need to store the entire TransformStorage object in any of these classes?

/** \brief A CountCache object backed up by files on disk for long duration transform storage.
 * *//*
class DiskCache : public TimeCacheInterface
{
public:

  /// Output
  DiskCache(const std::string &output_prefix, const size_t max_file_size_bytes=104857600,
            const size_t max_cache_entries=10000);
  ~DiskCache();

  /// Access data from the cache
  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0)
  {
    if (memory_cache_.getData(time, data_out, error_str))
      return true; // The data was already cached in memory
    TODO load from disk into memory, then retry from memory
  }

  /// Insert data into the cache
  virtual bool insertData(const TransformStorage& new_data)
  {
    //memory_cache_.insertData(new_data);
    TODO write to disk
  }

  /// Clear the list of stored values
  virtual void clearList() 
  {
    // Don't clear the disk data, just what is cached in memory.
    memory_cache.clearList();
  }

  /// Retrieve the parent at a specific time
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str)
  {
    if (memory_cache(time, error_str))
      return true;
    TODO load from disk into memory, then retry from memory
  }

  ///Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
  virtual P_TimeAndFrameID getLatestTimeAndParent() 
  {
    return P_TimeAndFrameID(); // TODO: Does this have to be supported?
  }


  /// - Debugging information methods

  /// Get the length of the stored list
  virtual unsigned int getListLength()
  {
    TODO
  }

  /// Get the latest timestamp cached
  virtual ros::Time getLatestTimestamp()
  {
    TODO
  }

  /// Get the oldest timestamp cached
  virtual ros::Time getOldestTimestamp()
  {
    TODO
  }

private: // Variables

  /// Names of all the known files on disk
  std::vector<std::string> files_on_disk_;

  /// Memory based portion of the cache
  CountCache memory_cache_;

  /// Path prefix for files written to disk
  std::string disk_prefix_;

  /// Try to keep output log files smaller than this
  size_t max_file_size_bytes_;

  /// Keep a handle open for writing
  std::ofstream file_write_handle;

  /// The size of the file opened with file_write_handle
  size_t output_file_size_;

  // TODO: Use a mutex to prevent multiple access

private: // Functions


  /// Find all of the files already written to disk at the specified prefix
  size_t findExistingFiles(std::vector<std::string> &file_list)
  {
    // TODO: Get all the files that start with disk_prefix_
    return 0;
  }

  // TODO: More file IO error checking

  // TODO: How to log entries to disk if they can come in out of order?
  //       -> Seems like only a proper database can really handle this well...

  bool openFileForNewEntry()
  {
    // If the current file handle is good just use it.
    if (file_write_handle_.is_open() && 
        (output_file_size_ < (max_file_size_bytes_ - sizeof(TransformStorage)))
      return true;

    // Otherwise close it and open a new one.
    if (file_write_handle_.is_open())
      file_write_handle.close();
    std::string new_file = getFileForWriting();
    file_write_handle.open(new_file, std::ofstream::binary);
    output_file_size_ = 0;
    return true;
  }

  /// Open the correct output file for writing
  std::string getFileForWriting()
  {
    std::vector<std::string> file_list;

    size_t num_files = findExistingFiles(file_list);
    if (num_files > 0)
    {
      size_t last_file_size = 0; // TODO: Get size of file_list.back() in bytes
      if (last_file_size < max_file_size_bytes_ - sizeof(TransformStorage))
        return file_list.back();
    }
    // If we get here will need to create a new file
    std::stringstream s;
    s << disk_prefix_ << "-" << num_files << ".dat";
    return s.str();
  }

  /// If available, load data around the selected time into the memory cache.
  bool diskToCache(ros::Time time, std::string* error_str = 0)
  {
  }

  /// Get the time span covered by a file on disk
  bool getTimeRange(const std::string &path, ros::Time &start, ros::Time &end)
  {

  }

}; // End class DiskCache

*/

}

#endif // TF2_TIME_CACHE_H
