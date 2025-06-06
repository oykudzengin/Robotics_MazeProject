// File: play_song1.cpp
// Description: Store and play only song 1 on the iRobot Create via ROS services.

#include <ros/ros.h>
#include <create_fundamentals/StoreSong.h>
#include <create_fundamentals/PlaySong.h>
#include <vector>

int main(int argc, char** argv)
{
    // Initialize the ROS node as "play_song1_node"
    ros::init(argc, argv, "play_song1_node");
    ros::NodeHandle nh;

    // ─── 1. Wait for the Create's 'store_song' service ───────────────────────────
    ROS_INFO("Waiting for 'store_song' service to appear...");
    ros::service::waitForService("store_song");
    ros::ServiceClient store_client =
        nh.serviceClient<create_fundamentals::StoreSong>("store_song");

    // ─── 2. Wait for the Create's 'play_song' service ────────────────────────────
    ROS_INFO("Waiting for 'play_song' service to appear...");
    ros::service::waitForService("play_song");
    ros::ServiceClient play_client =
        nh.serviceClient<create_fundamentals::PlaySong>("play_song");

    // ─── 3. Define the note‐to‐integer mappings (MIDI numbers) ────────────────────
    // Only the notes needed for song 1 are defined here:
    const uint32_t A4 = 69;   // MIDI note number for A4
    const uint32_t F4 = 65;   // MIDI note number for F4
    const uint32_t C5 = 72;   // MIDI note number for C5

    // ─── 4. Define time‐durations for each note (in multiples of 1/64 sec) ───────
    // We choose MEASURE = 160 so that:
    //  • a quarter note   → MEASURE/4   = 40 (i.e. 40/64 s ≈ 0.625 s)
    //  • a dotted eighth  → (MEASURE*3)/16 = 30 (30/64 s ≈ 0.469 s)
    //  • a sixteenth note → MEASURE/16  = 10 (10/64 s ≈ 0.156 s)
    //  • a half note      → MEASURE/2   = 80 (80/64 s = 1.25 s)
    const int    MEASURE      = 160;
    const uint8_t Q           = MEASURE / 4;       // Quarter note  (40/64 s)
    const uint8_t Ed          = (MEASURE * 3) / 16; // Dotted eighth  (30/64 s)
    const uint8_t S           = MEASURE / 16;      // Sixteenth note (10/64 s)
    const uint8_t HALF        = MEASURE / 2;       // Half note     (80/64 s)
    const double MEASURE_TIME = MEASURE / 64.0;     // 2.5 seconds per full measure

    // ─── 5. Prepare the data for “song 1” ─────────────────────────────────────────
    // This matches exactly what was in example_song.py for slot 1:
    //   a4, Q, a4, Q, a4, Q, f4, Ed, c5, S,
    //   a4, Q, f4, Ed, c5, S, a4, HALF
    std::vector<uint32_t> song1_data = {
        A4,  Q,  A4,  Q,  A4,  Q,  F4,  Ed, C5,  S,
        A4,  Q,  F4,  Ed, C5,  S,  A4,  HALF
    };

    // ─── 6. Call the 'store_song' service to upload song 1 into slot 1 ───────────
    create_fundamentals::StoreSong store_srv;
    store_srv.request.number = 1;
    store_srv.request.song = song1_data;

    ROS_INFO("Uploading (storing) song 1 to the Create...");
    if (!store_client.call(store_srv)) {
        ROS_ERROR("Failed to call service 'store_song'. Aborting.");
        return 1;
    }
    ROS_INFO("Song 1 was stored successfully.");

    // Add a short delay to ensure the Create’s driver has time to process the request.
    ros::Duration(0.2).sleep();

    // ─── 7. Call the 'play_song' service to play slot 1 ───────────────────────────
    create_fundamentals::PlaySong play_srv;
    play_srv.request.number = 1;

    ROS_INFO("Now playing song 1...");
    if (!play_client.call(play_srv)) {
        ROS_ERROR("Failed to call service 'play_song'. Aborting.");
        return 1;
    }

    // Sleep long enough for the entire melody to finish (≈ 2 measures × 2.5 s each).
    ros::Duration(MEASURE_TIME * 2.01).sleep();

    ROS_INFO("Finished playing song 1. Exiting node.");
    return 0;
}