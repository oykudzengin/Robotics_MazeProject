#pragma once

#include <ros/ros.h>
#include <cstdint>
#include <vector>

namespace silver_fundamentals {

/**
 * @brief Helper to store and play a song on a given slot.
 * @param nh ROS NodeHandle used to create service clients.
 * @param slot Slot number to store/play the song.
 * @param songData Vector of MIDI note numbers interleaved with durations.
 * @return true on success, false on failure.
 */
bool storeAndPlaySong(ros::NodeHandle& nh,
                      uint8_t slot,
                      const std::vector<uint32_t>& songData);

/**
 * @brief Stores and plays song 1 (slot 1).
 * @param nh ROS NodeHandle used to create service clients.
 */
void playSong1(ros::NodeHandle& nh);

/**
 * @brief Stores and plays song 2 (slot 2).
 * @param nh ROS NodeHandle used to create service clients.
 */
void playSong2(ros::NodeHandle& nh);

/**
 * @brief Stores and plays song 3 (slot 3).
 */
void playSong3(ros::NodeHandle& nh);

/**
 * @brief Stores and plays song 4 (slot 4).
 */
void playSong4(ros::NodeHandle& nh);

} // namespace silver_fundamentals
