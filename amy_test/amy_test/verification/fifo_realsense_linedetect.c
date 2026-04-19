/*
 *  Real-time Line Detection Performance Test - SCHED_FIFO
 *  Purpose: Test Hough line detection on RealSense camera for line following
 *  
 *  Tests: Hough Lines at 3 resolutions (160×120, 320×240, 640×480)
 *  Scheduler: SCHED_FIFO (priority 90)
 *  Frames per test: 100
 *  Deadline: 50 ms (target for 20 Hz control loop)
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sched.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <iostream>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#define MAX_FRAMES 100
#define DEADLINE_US 50000LL  // 50ms deadline for line following control loop

// Set to 0 to disable GUI display (for headless systems)
#define ENABLE_GUI 1

// Resolution configurations
typedef struct {
    int width;
    int height;
    const char* name;
} Resolution;

Resolution resolutions[] = {
    {320, 240, "320x240"},
    {424, 240, "424x240"},  // RealSense common resolution
    {640, 480, "640x480"}
};
#define NUM_RESOLUTIONS 3

// Global variables for Hough line detection
Mat frame_mat, gray, edges, output;
vector<Vec4i> detected_lines;

static long long ts_to_us(const struct timespec *ts)
{
    return ((long long)ts->tv_sec * 1000000LL) + (ts->tv_nsec / 1000LL);
}

void apply_hough_lines_transform(Mat& input, Mat& output_vis)
{
    // Convert to grayscale
    cvtColor(input, gray, COLOR_BGR2GRAY);
    
    // Apply Canny edge detection
    Canny(gray, edges, 50, 150, 3);
    
    // Detect lines using Hough transform
    detected_lines.clear();
    HoughLinesP(edges, detected_lines, 1, CV_PI/180, 50, 50, 10);
    
    // Store edges for visualization (done after timing)
    cvtColor(edges, output_vis, COLOR_GRAY2BGR);
}

void draw_hough_lines(Mat& output_vis)
{
    for(size_t i = 0; i < detected_lines.size(); i++)
    {
        Vec4i l = detected_lines[i];
        line(output_vis, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }
}

void run_test(rs2::pipeline& pipe, Resolution res)
{
    char csv_name[256];
    char window_name[256];
    
    snprintf(csv_name, sizeof(csv_name), "fifo_realsense_houghlines_%s.csv", res.name);
    snprintf(window_name, sizeof(window_name), "FIFO RealSense - Hough Lines - %s", res.name);
    
    printf("\n========================================\n");
    printf("Test: Hough Lines at %s\n", res.name);
    printf("CSV: %s\n", csv_name);
    printf("========================================\n");
    
    FILE *csv_fp = fopen(csv_name, "w");
    if(csv_fp == NULL)
    {
        printf("ERROR: could not open CSV file %s\n", csv_name);
        return;
    }
    
    // Configure RealSense pipeline
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, res.width, res.height, RS2_FORMAT_BGR8, 30);
    
    try {
        pipe.start(cfg);
    } catch (const rs2::error & e) {
        printf("ERROR: Failed to start pipeline: %s\n", e.what());
        fclose(csv_fp);
        return;
    }
    
    printf("RealSense: %dx%d @ 30 FPS\n", res.width, res.height);
    
    // Wait for camera to stabilize
    printf("Warming up camera...\n");
    for(int i = 0; i < 30; i++) {
        pipe.wait_for_frames();
    }
    
#if ENABLE_GUI
    namedWindow(window_name, WINDOW_AUTOSIZE);
#endif
    
    int frame_count = 0;
    int miss_count = 0;
    
    struct timespec start_ts, end_ts;
    long long start_us, end_us, exec_us, prev_end_us = 0, delta_us;
    int miss;
    
    long long exec_sum_us = 0;
    long long delta_sum_us = 0;
    long long exec_max_us = 0;
    long long delta_max_us = 0;
    long long exec_min_us = LLONG_MAX;
    long long delta_min_us = LLONG_MAX;
    double exec_sum_sq = 0.0;
    double delta_sum_sq = 0.0;
    int stats_count = 0;
    
    fprintf(csv_fp, "frame,start_us,end_us,exec_us,delta_us,deadline_us,miss,lines_detected\n");
    
    while(frame_count < MAX_FRAMES)
    {
        clock_gettime(CLOCK_MONOTONIC, &start_ts);
        
        // Wait for frame from RealSense
        rs2::frameset frames;
        try {
            frames = pipe.wait_for_frames(5000); // 5 second timeout
        } catch (const rs2::error & e) {
            printf("ERROR: Frame capture failed: %s\n", e.what());
            break;
        }
        
        rs2::frame color_frame = frames.get_color_frame();
        if (!color_frame) {
            printf("ERROR: No color frame available\n");
            break;
        }
        
        // Convert RealSense frame to OpenCV Mat
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();
        frame_mat = Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        
        // Apply Hough line detection (timed portion)
        apply_hough_lines_transform(frame_mat, output);
        
        // End timing (excludes visualization and display overhead)
        clock_gettime(CLOCK_MONOTONIC, &end_ts);
        
        int num_lines = detected_lines.size();
        
        // Draw visualization overlays (after timing)
        draw_hough_lines(output);
        
#if ENABLE_GUI
        // Display (not counted in RT performance)
        imshow(window_name, output);
#endif
        
        start_us = ts_to_us(&start_ts);
        end_us = ts_to_us(&end_ts);
        exec_us = end_us - start_us;
        
        if(frame_count == 0)
        {
            delta_us = 0;
            miss = 0;
        }
        else
        {
            delta_us = end_us - prev_end_us;
            miss = (delta_us > DEADLINE_US) ? 1 : 0;
        }
        
        fprintf(csv_fp, "%d,%lld,%lld,%lld,%lld,%lld,%d,%d\n",
                frame_count, start_us, end_us, exec_us, delta_us, DEADLINE_US, miss, num_lines);
        
        if(frame_count > 0)
        {
            exec_sum_us += exec_us;
            delta_sum_us += delta_us;
            exec_sum_sq += (double)exec_us * (double)exec_us;
            delta_sum_sq += (double)delta_us * (double)delta_us;
            
            if(exec_us > exec_max_us)
                exec_max_us = exec_us;
            
            if(exec_us < exec_min_us)
                exec_min_us = exec_us;
            
            if(delta_us > delta_max_us)
                delta_max_us = delta_us;
            
            if(delta_us < delta_min_us)
                delta_min_us = delta_us;
            
            miss_count += miss;
            stats_count++;
        }
        
        prev_end_us = end_us;
        frame_count++;
        
#if ENABLE_GUI
        // Allow 33ms for display refresh and key check (~30 FPS)
        char q = waitKey(33);
        if(q == 'q')
        {
            printf("User quit at frame %d\n", frame_count);
            break;
        }
#else
        // Progress indicator for headless mode
        if(frame_count % 10 == 0) {
            printf("  Frame %d/%d\r", frame_count, MAX_FRAMES);
            fflush(stdout);
        }
#endif
    }
    
    // Write summary statistics
    if(stats_count > 0)
    {
        printf("\n");  // Newline after progress indicator
        double avg_exec_us = (double)exec_sum_us / (double)stats_count;
        double avg_delta_us = (double)delta_sum_us / (double)stats_count;
        double fps = 1000000.0 / avg_delta_us;
        double miss_percent = (100.0 * (double)miss_count) / (double)stats_count;
        
        // Calculate standard deviation
        double exec_variance = (exec_sum_sq / (double)stats_count) - (avg_exec_us * avg_exec_us);
        double delta_variance = (delta_sum_sq / (double)stats_count) - (avg_delta_us * avg_delta_us);
        double exec_std = (exec_variance > 0) ? sqrt(exec_variance) : 0.0;
        double delta_std = (delta_variance > 0) ? sqrt(delta_variance) : 0.0;
        
        // Calculate jitter
        long long exec_jitter = exec_max_us - exec_min_us;
        long long delta_jitter = delta_max_us - delta_min_us;
        
        // Calculate coefficient of variation
        double exec_cv = (avg_exec_us > 0) ? (exec_std / avg_exec_us) : 0.0;
        double delta_cv = (avg_delta_us > 0) ? (delta_std / avg_delta_us) : 0.0;
        
        printf("\n----- Timing Summary (excluding frame 0) -----\n");
        printf("Frames analyzed: %d\n", stats_count);
        printf("\n--- Execution Time Statistics ---\n");
        printf("  Mean:        %8.2f μs  (%6.2f ms)\n", avg_exec_us, avg_exec_us / 1000.0);
        printf("  Std Dev:     %8.2f μs  (%6.2f ms)\n", exec_std, exec_std / 1000.0);
        printf("  Min:         %8lld μs  (%6.2f ms)\n", exec_min_us, exec_min_us / 1000.0);
        printf("  Max:         %8lld μs  (%6.2f ms)\n", exec_max_us, exec_max_us / 1000.0);
        printf("  Jitter:      %8lld μs  (%6.2f ms)\n", exec_jitter, exec_jitter / 1000.0);
        printf("  CV:          %8.4f     (%6.2f %%)\n", exec_cv, exec_cv * 100.0);
        
        printf("\n--- Frame Rate Statistics ---\n");
        printf("  Average FPS: %8.2f\n", fps);
        printf("  Delta Mean:  %8.2f μs  (%6.2f ms)\n", avg_delta_us, avg_delta_us / 1000.0);
        printf("  Delta Std:   %8.2f μs  (%6.2f ms)\n", delta_std, delta_std / 1000.0);
        printf("  Delta Jitter:%8lld μs  (%6.2f ms)\n", delta_jitter, delta_jitter / 1000.0);
        printf("  Delta CV:    %8.4f     (%6.2f %%)\n", delta_cv, delta_cv * 100.0);
        
        printf("\n--- Deadline Analysis ---\n");
        printf("  Deadline:    %8lld μs  (%6.2f ms)\n", DEADLINE_US, DEADLINE_US / 1000.0);
        printf("  Misses:      %8d / %d\n", miss_count, stats_count);
        printf("  Miss %%:      %8.2f %%\n", miss_percent);
        
        if(miss_percent < 1.0)
            printf("  ✓ Excellent real-time performance\n");
        else if(miss_percent < 5.0)
            printf("  ✓ Good real-time performance\n");
        else if(miss_percent < 10.0)
            printf("  ⚠ Marginal real-time performance\n");
        else
            printf("  ✗ Poor real-time performance\n");
        
        printf("----------------------------------------------\n");
        
        fprintf(csv_fp, "\n");
        fprintf(csv_fp, "# Summary (excluding frame 0)\n");
        fprintf(csv_fp, "# Frames analyzed,%d\n", stats_count);
        fprintf(csv_fp, "# Average exec time us,%.3f\n", avg_exec_us);
        fprintf(csv_fp, "# Exec std dev us,%.3f\n", exec_std);
        fprintf(csv_fp, "# Min exec time us,%lld\n", exec_min_us);
        fprintf(csv_fp, "# Max exec time us,%lld\n", exec_max_us);
        fprintf(csv_fp, "# Exec jitter us,%lld\n", exec_jitter);
        fprintf(csv_fp, "# Exec CV,%.6f\n", exec_cv);
        fprintf(csv_fp, "# Average frame delta us,%.3f\n", avg_delta_us);
        fprintf(csv_fp, "# Delta std dev us,%.3f\n", delta_std);
        fprintf(csv_fp, "# Min frame delta us,%lld\n", delta_min_us);
        fprintf(csv_fp, "# Max frame delta us,%lld\n", delta_max_us);
        fprintf(csv_fp, "# Delta jitter us,%lld\n", delta_jitter);
        fprintf(csv_fp, "# Delta CV,%.6f\n", delta_cv);
        fprintf(csv_fp, "# Estimated FPS,%.3f\n", fps);
        fprintf(csv_fp, "# Deadline us,%lld\n", DEADLINE_US);
        fprintf(csv_fp, "# Deadline misses,%d\n", miss_count);
        fprintf(csv_fp, "# Miss percentage,%.2f\n", miss_percent);
    }
    
    fclose(csv_fp);
#if ENABLE_GUI
    destroyWindow(window_name);
#endif
    pipe.stop();
    
    printf("✓ Saved: %s\n", csv_name);
}

int main(int argc, char** argv)
{
    printf("\n======================================\n");
    printf("REALSENSE LINE DETECTION - FIFO TEST\n");
    printf("======================================\n");
    printf("Purpose: Measure Hough line detection performance\n");
    printf("         for real-time line following control\n");
    printf("Resolutions: %d (320×240, 424×240, 640×480)\n", NUM_RESOLUTIONS);
    printf("Frames per test: %d\n", MAX_FRAMES);
    printf("Deadline: %lld ms (for 20 Hz control loop)\n", DEADLINE_US / 1000);
#if ENABLE_GUI
    printf("Press 'q' during a test to skip to next\n");
#else
    printf("Running in HEADLESS mode (no GUI display)\n");
#endif
    printf("======================================\n");
    
    // Set SCHED_FIFO real-time scheduler
    struct sched_param param;
    param.sched_priority = 90;
    
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("ERROR: sched_setscheduler failed (need sudo/root)");
        printf("Continuing with default scheduler...\n");
    }
    else
    {
        printf("✓ Successfully set SCHED_FIFO scheduler (priority %d)\n", param.sched_priority);
    }
    
    // Initialize RealSense pipeline
    rs2::pipeline pipe;
    
    // Run tests for all resolutions
    int test_num = 1;
    int successful_tests = 0;
    for(int r = 0; r < NUM_RESOLUTIONS; r++)
    {
        printf("\n>>> Test %d/%d <<<\n", test_num++, NUM_RESOLUTIONS);
        try {
            run_test(pipe, resolutions[r]);
            successful_tests++;
        } catch (const rs2::error & e) {
            printf("⚠ SKIPPED: Resolution not supported (%s)\n", e.what());
        } catch (const std::exception & e) {
            printf("⚠ SKIPPED: Error occurred (%s)\n", e.what());
        }
        
        // Small delay between tests
        usleep(1000000); // 1 second
    }
    
    printf("\n======================================\n");
    printf("ALL TESTS COMPLETE!\n");
    printf("======================================\n");
    printf("Successful tests: %d/%d\n", successful_tests, NUM_RESOLUTIONS);
    
    if(successful_tests == 0) {
        printf("\nERROR: No tests completed successfully\n");
        printf("Check camera connection: rs-enumerate-devices\n");
        return 1;
    }
    
    printf("\nGenerated CSV files:\n");
    for(int r = 0; r < NUM_RESOLUTIONS; r++)
    {
        char csv_name[256];
        snprintf(csv_name, sizeof(csv_name), "fifo_realsense_houghlines_%s.csv", resolutions[r].name);
        FILE* check = fopen(csv_name, "r");
        if(check) {
            printf("  ✓ %s\n", csv_name);
            fclose(check);
        }
    }
    printf("\nAnalysis:\n");
    printf("  1. Check 'Mean' exec time - should be <50ms for real-time\n");
    printf("  2. Check 'Miss %%' - should be <5%% for good performance\n");
    printf("  3. Compare resolutions to find best speed/quality trade-off\n");
    printf("  4. Use selected resolution for ROS line_follow_V2 node\n");
    
    return 0;
}
