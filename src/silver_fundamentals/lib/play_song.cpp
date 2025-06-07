// File: play_songs.cpp
// Description: Defines reusable functions to store and play songs on the iRobot Create via ROS services.

#include <ros/ros.h>
#include <create_fundamentals/StoreSong.h>
#include <create_fundamentals/PlaySong.h>
#include <vector>
#include <cstdint>
#include <playsong.h>

namespace silver_fundamentals {

// Note durations (multiples of 1/64 sec)
static constexpr int MEASURE      = 160;        // full measure (2.5 s)
static constexpr uint8_t Q        = MEASURE / 4;       // quarter note
static constexpr uint8_t Ed       = (MEASURE * 3) / 16; // dotted eighth note
static constexpr uint8_t S        = MEASURE / 16;      // sixteenth note
static constexpr double MEASURE_TIME = MEASURE / 64.0; // seconds per measure

/**
 * @brief Helper to store and play a song on a given slot.
 * @param nh ROS NodeHandle used to create service clients.
 * @param slot Slot number to store/play the song.
 * @param songData Vector of MIDI note numbers interleaved with durations.
 * @return true on success, false on failure.
 */
bool storeAndPlaySong(ros::NodeHandle& nh,
                      uint8_t slot,
                      const std::vector<uint32_t>& songData)
{
    // Wait for services
    ros::service::waitForService("store_song");
    ros::service::waitForService("play_song");

    ros::ServiceClient store_client = 
        nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    ros::ServiceClient play_client = 
        nh.serviceClient<create_fundamentals::PlaySong>("play_song");

    // Store the song
    create_fundamentals::StoreSong store_srv;
    store_srv.request.number = slot;
    store_srv.request.song   = songData;
    if (!store_client.call(store_srv)) {
        ROS_ERROR("Failed to store song in slot %d", slot);
        return false;
    }
    ros::Duration(0.2).sleep();  // allow driver to process

    // Play the song
    create_fundamentals::PlaySong play_srv;
    play_srv.request.number = slot;
    if (!play_client.call(play_srv)) {
        ROS_ERROR("Failed to play song in slot %d", slot);
        return false;
    }

    // Wait long enough for song to finish (≈ 2 measures)
    ros::Duration(MEASURE_TIME * 2.01).sleep();
    return true;
}

/**
 * @brief Stores and plays song 1 (slot 1).
 * @param nh ROS NodeHandle used to create service clients.
 */
void playSong1(ros::NodeHandle& nh)
{
    // Song 1 data: a4, Q, a4, Q, a4, Q, f4, Ed, c5, S, a4, Q, f4, Ed, c5, S, a4, HALF
    constexpr uint32_t A4 = 69;
    constexpr uint32_t F4 = 65;
    constexpr uint32_t C5 = 72;
    constexpr uint8_t HALF = MEASURE / 2;

    std::vector<uint32_t> song1_data = {
        A4, Q,  A4, Q,  A4, Q,   F4, Ed,
        C5, S,  A4, Q,  F4, Ed,  C5, S,
        A4, HALF
    };

    if (!storeAndPlaySong(nh, 1, song1_data)) {
        ROS_ERROR("playSong1 failed");
    }
}

/**
 * @brief Stores and plays song 2 (slot 2).
 * @param nh ROS NodeHandle used to create service clients.
 */
void playSong2(ros::NodeHandle& nh)
{
    // Song 2 data: e5, Q, e5, Q, e5, Q, f5, Ed, c5, S, aes4, Q, f4, Ed, c5, S, a4, HALF
    constexpr uint32_t E5   = 76;
    constexpr uint32_t F5   = 77;
    constexpr uint32_t C5   = 72;
    constexpr uint32_t AES4 = 68;
    constexpr uint32_t F4   = 65;
    constexpr uint32_t A4   = 69;
    constexpr uint8_t  HALF = MEASURE / 2;

    std::vector<uint32_t> song2_data = {
        E5, Q,   E5, Q,   E5, Q,   F5, Ed,
        C5, S,   AES4, Q,  F4, Ed,  C5, S,
        A4, HALF
    };



    if (!storeAndPlaySong(nh, 2, song2_data)) {
        ROS_ERROR("playSong2 failed");
    }
}

/**
 * @brief Stores and plays the Windows shutdown tone (slot 3).
 * @param nh ROS NodeHandle used to create service clients.
 */
void playSong3(ros::NodeHandle& nh)
{
    // Windows shutdown tone notes and durations
    // Approximate MIDI notes and durations for the Windows shutdown sound
    constexpr uint32_t A4 = 69;
    constexpr uint32_t G4 = 67;
    constexpr uint32_t F4 = 65;
    constexpr uint8_t Q = MEASURE / 4;       // quarter note
    constexpr uint8_t H = MEASURE / 2;       // half note
    constexpr uint8_t E = MEASURE / 8;       // eighth note

    std::vector<uint32_t> song3_data = {
        A4, H,
        G4, H,
        F4, Q,
        G4, E,
        A4, H
    };

    if (!storeAndPlaySong(nh, 3, song3_data)) {
        ROS_ERROR("playSong3 failed");
    }
}
} // namespace silver_fundamentals

int main(int argc, char** argv) {
    ros::init(argc, argv, "play_song_node");
    ros::NodeHandle nh;

    // Pick which song to play; for example:
    silver_fundamentals::playSong1(nh);
    silver_fundamentals::playSong2(nh);
    silver_fundamentals::playSong3(nh);
    
    // or: playSong2(nh);

    return 0;
}