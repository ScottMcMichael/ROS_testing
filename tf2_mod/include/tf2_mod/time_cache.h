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

#ifndef TF2_MOD_TIME_CACHE_H
#define TF2_MOD_TIME_CACHE_H

#include "tf2/transform_storage.h"
#include "tf2/time_cache.h"

#include <deque>
#include <queue>

#include <sstream>

#include <ros/message_forward.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

#include <sqlite3.h> // TODO: Hide in the CPP file?

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
 * data out as a function of time. */
class TimeCache2 : public tf2::TimeCacheInterface
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache2(ros::Duration  max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));


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
  

protected:
  typedef std::deque<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  ros::Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two, ros::Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);


  void pruneList();



};
/*
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
 */
class CountCache : public tf2_mod::TimeCache2
{
 public:

  CountCache(size_t max_entries = 10000); // TODO: Adjust the default length

  // The public interface is the same as the TimeCache class.

  virtual bool insertData(const TransformStorage& new_data);


private:

  size_t  max_num_entries_;

  /// Record the data timestampss in the order they are inserted.
  std::queue<ros::Time> insertion_order_queue_;
  
  // Override the prune behavior from TimeCache
  void pruneList();

}; // End class CountCache

// TODO: Move out of ROS!
// TODO: Need to store a "version number" in the database to allow point updates
//       - Need to record the time of entry in addition to the version number?
//       -> Current plan is that upates must always be for an existing timestamp/entry!
//       - Separate table for updates?
//       - Modifications can't be through the TF interface, need another process to put these in.
//       - Need to make sure that modifications get to the right file/table
// TODO: Better error handling
// TODO: Verify that the time conversions are repeatable!
/** \brief A Cache object backed up by files on disk for long duration transform storage.
 *  - Each DiskCache object is locked to a certain parent/child frame combination.
 *  - Each parent/child frame combination must have a different file on disk.
 *  - The in-memory cache does not look at updates inserted into the disk database.
 *    Because of this, no updates should be committed until after max_storage_time
 *    has elapsed after the data was initially published.
 * */
class DiskCache : public tf2::TimeCacheInterface
{
public:

  static const int64_t DEFAULT_MAX_STORAGE_TIME = 300ULL * 100000000LL; //!< default value of 5 minutes storage
  static const size_t TEMP_CACHE_MAX_SIZE = 5000;

  // The ID's are required here because they are constant for each Cache object.
  DiskCache(CompactFrameID parent_id,
            CompactFrameID child_id,
            const std::string &disk_path="/home/smcmich1/dummy.db",
            const ros::Duration max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME))
            :memory_cache_(max_storage_time), temporary_cache_(TEMP_CACHE_MAX_SIZE),
            disk_path_(disk_path),
             parent_frame_id_(parent_id), child_frame_id_(child_id) {}
  ~DiskCache()
  {
    // Disconnect from the SQLite database
    int result = sqlite3_close(db_);
    if (!result)
      std::cout << "Error code " << result << " when closing SQL connection.\n";
  }

  /// Prepare for interaction with disk.
  bool initialize()
  {
    // Connect to the SQLite database
    // - Create the file if it does not exist.
    int result = sqlite3_open_v2(disk_path_.c_str(), &db_,
                                 SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
    if (result != SQLITE_OK)
    {
      std::cout << "Error on disk init: " << disk_path_
                << "  -->  " << result << std::endl;
      return false;
    }

    // Define all the columns that will make up the table
    std::vector<std::string> columns;
    //columns.push_back("time real");
    columns.push_back("sec integer");
    columns.push_back("nsec integer");   
    columns.push_back("tx real");
    columns.push_back("ty real");
    columns.push_back("tz real");
    columns.push_back("qx real");
    columns.push_back("qy real");
    columns.push_back("qz real");
    columns.push_back("qw real");
    
    std::string table_init_string = "CREATE TABLE IF NOT EXISTS frame_data (";
    for (size_t i=0; i<columns.size()-1; ++i)
      table_init_string += columns[i] + ", ";
    table_init_string += columns.back() + ")";

    //TODO: Error handling
    return executeSqlCommand(table_init_string);
  }

  /// Access data from the cache
  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0)
  {
    // Try from memory
    if (memory_cache_.getData(time, data_out, error_str))
      return true;
    std::cout << "Attempting disk retrieval.\n";
    // Try to move the data from disk to the temporary cache
    if (!diskToCache(time))
      return false;

    // Try to process from the temporary cache
    bool result = temporary_cache_.getData(time, data_out, error_str);
    temporary_cache_.clearList();
    return result;
  }

  /// Insert data into the cache
  virtual bool insertData(const TransformStorage& new_data)
  {
    // Verify that these are the expected frame ID's
    if ((new_data.frame_id_       != parent_frame_id_) ||
        (new_data.child_frame_id_ != child_frame_id_ )   )
    {
      std::cout << "DiskCache ID mismatch!\n";
      return false;
    }

    // Local memory insert
    memory_cache_.insertData(new_data);

    // Now insert into the SQL database
    std::stringstream s;
    //s.precision(20);
    s << "INSERT INTO frame_data VALUES (";
    s << new_data.stamp_.sec;
    s << ", " << new_data.stamp_.nsec;
    s << ", " << new_data.translation_.getX();
    s << ", " << new_data.translation_.getY();
    s << ", " << new_data.translation_.getZ();
    s << ", " << new_data.rotation_.getAxis().x();
    s << ", " << new_data.rotation_.getAxis().y();
    s << ", " << new_data.rotation_.getAxis().z();
    s << ", " << new_data.rotation_.getW() << ")";
    return executeSqlCommand(s.str());
  }

  /// Clear the list of cached values (not disk data)
  virtual void clearList() 
  {
    memory_cache_.clearList();
  }

  /// Retrieve the parent at a specific time
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str)
  {
    return parent_frame_id_; // Not sure why this is a function...
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
    return 0; // TODO: Does this have to be supported?
  }

  /// Get the latest timestamp cached
  virtual ros::Time getLatestTimestamp()
  {
    return ros::Time(); // TODO: Does this have to be supported?
  }

  /// Get the oldest timestamp cached
  virtual ros::Time getOldestTimestamp()
  {
    return ros::Time(); // TODO: Does this have to be supported?
  }



/*
  // These functions can't help but lose accuracy!
  /// Convert from the two-int ros::Time to a double used by SQLite
  static double rosTimeToSqliteTime(const ros::Time t)
  {
    const uint32_t SECONDS_PER_DAY   = 86400;
    const double   SECONDS_PER_DAY_D = 86400.0;
    const double UNIX_START      = 2440587.5; // Unix epoch time in seconds
    const double NANOSECONDS_PER_SECOND = 1000000000;

    uint32_t days           = t.sec / SECONDS_PER_DAY;
    uint32_t remaining_secs = t.sec % SECONDS_PER_DAY;

    double julian_day = static_cast<double>(days) + 
                static_cast<double>(remaining_secs) / SECONDS_PER_DAY_D
                + UNIX_START;

    //double julian_day = (t.sec / SECONDS_PER_DAY) + UNIX_START;
    double fractional = static_cast<double>(t.nsec) / NANOSECONDS_PER_SECOND;
    return julian_day + fractional;
  }

  /// Convert from the two-int ros::Time to a double used by SQLite
  static ros::Time sqliteTimeToRosTime(const double t)
  {
    const double SECONDS_PER_DAY = 86400.0;
    const double UNIX_START      = 2440587.5;
    const double NANOSECONDS_PER_SECOND = 1000000000;

    double seconds = (t - UNIX_START) * SECONDS_PER_DAY;
    double whole_seconds = floor(seconds);
    double fraction = seconds - whole_seconds;

    ros::Time rt;
    rt.sec  = seconds;
    rt.nsec = fraction * NANOSECONDS_PER_SECOND;
    return rt;
  }
*/


private: // Variables

  /// Memory based portion of the cache.
  /// - Never gets updated information from the database.
  tf2::TimeCache memory_cache_;

  /// Cache object used only for immediate interpolation of disk data.
  /// - Cleared after each time it is used so that it never has "stale" data
  ///   that is missing a recent update in the disk database.
  CountCache temporary_cache_;

  /// Location of the database file on disk
  std::string disk_path_;

  /// Database handle.
  sqlite3 *db_;

  // The parent and child frames are fixed on initialization.
  tf2::CompactFrameID parent_frame_id_;
  tf2::CompactFrameID child_frame_id_;

private: // Functions

  // TODO: Use the convenience wrappers to reduce the code volume here

  // TODO: Bulk up table inserts then do them all as part of a transaction
  //       - Must insert after a timeout!



  /// Execute an SQL command which expects no outputs
  bool executeSqlCommand(const std::string &command)
  {
    // Prepare the SQL command.
    sqlite3_stmt *ppStmt;
    int result = sqlite3_prepare_v2(db_, command.c_str(), -1, &ppStmt, 0);
    if (result != SQLITE_OK)
    {
      std::cout << "Error executing command " << command << std::endl;
      return false;
    }
    else
      std::cout << "Executed command " << command << std::endl;

    // Execute the command    
    result = sqlite3_step(ppStmt);
    if (result != SQLITE_DONE)
      std::cout << "Error code " << result << " following SQL command: "
                << command << std::endl;

    // Clean up the command
    result = sqlite3_finalize(ppStmt);
    if (result != SQLITE_OK)
      std::cout << "Error code " << result << " cleaning up SQL command: "
                << command << std::endl;

    return (result == SQLITE_OK);
  }

  /// Unpacks a single row from an SQL result into a TransformStorage object.
  void getDataFromSqlRow(sqlite3_stmt *ppStmt, TransformStorage &ts)
  {
    double tx, ty, tz, rx, ry, rz, rw, sqlTime;
    ts.stamp_.sec      = sqlite3_column_int(ppStmt, 0);
    ts.stamp_.nsec     = sqlite3_column_int(ppStmt, 1);
    ts.frame_id_       = parent_frame_id_;
    ts.child_frame_id_ = child_frame_id_;
    tx = sqlite3_column_double(ppStmt, 2);
    ty = sqlite3_column_double(ppStmt, 3);
    tz = sqlite3_column_double(ppStmt, 4);
    rx = sqlite3_column_double(ppStmt, 5);
    ry = sqlite3_column_double(ppStmt, 6);
    rz = sqlite3_column_double(ppStmt, 7);
    rw = sqlite3_column_double(ppStmt, 8);

    ts.translation_ = tf2::Vector3(tx, ty, tz);
    ts.rotation_    = tf2::Quaternion(rx, ry, rz, rw);
  }

  /// Send a data request command to the database and unpack the results.
  /// - Returns records.size()
  size_t getSqlRecords(const std::string &command, 
                       std::vector<TransformStorage> &records)
  {
    records.clear();

    std::cout << "Retrieving disks with command: " << command << std::endl;

    // Prepare the SQL command.
    sqlite3_stmt *ppStmt;
    int result = sqlite3_prepare_v2(db_, command.c_str(), -1, &ppStmt, 0);
    if (result != SQLITE_OK)
    {
      std::cout << "Error code: " << result << std::endl;
      return 0;
    }
    
    // Retrieve all of the results
    TransformStorage ts;
    result = sqlite3_step(ppStmt);
    while (result == SQLITE_ROW)
    {
      getDataFromSqlRow(ppStmt, ts);
      records.push_back(ts);

      result = sqlite3_step(ppStmt); // Move on to the next row
    }

    if (result != SQLITE_DONE)
      std::cout << "Error code " << result << " following SQL command: "
                << command << std::endl;

    // Clean up the command
    result = sqlite3_finalize(ppStmt);
    if (result != SQLITE_OK)
      std::cout << "Error code " << result << " cleaning up SQL command: "
                << command << std::endl;

    return records.size();
  }

  /// If available, load data around the selected time into the memory cache.
  size_t diskToCache(ros::Time rt)
  {
    // TODO: Break out this parameter!
    // Load this many entries in each direction from the target time
    // - Since we only search for data based on the whole second count,
    //   we need to request enough data points that we catch all of 
    //   the desired nsec points in our data range.  Currently we only
    //   interpolate from the two adjacent points so we only need to
    //   extend one second out in each direction.
    const int NUM_DESIRED_BORDERS = 10;

    int sec = rt.sec;

    // Assemble the query command
    // - Two queries, one to find N points before our time and
    //   another to find N points after it.
    std::stringstream s;
    s.precision(20);
    s << "SELECT * FROM (SELECT * FROM frame_data WHERE sec >= "
      << sec 
      << " ORDER BY sec ASC, nsec ASC LIMIT " << NUM_DESIRED_BORDERS
      << ") UNION SELECT * FROM (SELECT * FROM frame_data WHERE sec < "
      << sec << " ORDER BY sec DESC, nsec DESC LIMIT " 
      << NUM_DESIRED_BORDERS << ")";

    std::vector<TransformStorage> records;
    size_t num_records = getSqlRecords(s.str(), records);

    std::cout << "Retrieved " << num_records << " records from disk.\n";

    for (size_t i=0; i<num_records; ++i) 
    {
      temporary_cache_.insertData(records[i]);
      //std::cout << "Insert " << records[i].stamp_ << std::endl;
    }

    return num_records;
  }


}; // End class DiskCache



}

#endif // TF2_TIME_CACHE_H
