#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include "yolov8.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include "camera.h"
#include "galvo_control.h"

static void PrintDetectionResult(int frame_num, object_detect_result_list* od_results) {
    char frame_str[5];
    snprintf(frame_str, sizeof(frame_str), "%04d", frame_num);
    if (od_results->count == 0) {
        if (frame_num % 10 == 0)
            printf("【%s帧】未检测到杂草\n", frame_str);
        return;
    }
    printf("【%s帧】检测到 %d 个杂草：\n", frame_str, od_results->count);
    for (int i = 0; i < od_results->count; i++) {
        auto* d = &od_results->results[i];
        int cx = (d->box.left + d->box.right) / 2;
        int cy = (d->box.top + d->box.bottom) / 2;
        int w = d->box.right - d->box.left;
        int h = d->box.bottom - d->box.top;
        printf("  → 杂草%d: 置信度=%.1f%%, 中心=(%d,%d), 大小=%dx%d\n", i+1, d->prop*100, cx, cy, w, h);
    }
}

int main(int argc, char** argv) {
    int frame_num = 0;
    if (argc != 2) { printf("Usage: %s <model.rknn>\n", argv[0]); return -1; }
    const char* model_path = argv[1];
    int ret;

    HikCamera camera;
    if (!camera.Init()) { printf("Camera init failed\n"); return -1; }

    int galvo_fd = galvo_spi_open();
    if (galvo_fd < 0) printf("Galvo init failed\n");
    else { calib_load("calibration.dat"); printf("Galvo ready\n"); }

    pthread_t exit_thread;
    pthread_create(&exit_thread, NULL, (void*(*)(void*))PressEnterToExit, NULL);

    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    init_post_process();

    ret = init_yolov8_model(model_path, &rknn_app_ctx);
    if (ret != 0) { printf("Model load failed\n"); goto out; }

    printf("\nStarting... Press Enter to exit\n");

    while (!g_bExit) {
        image_buffer_t src_image;
        object_detect_result_list od_results;
        MV_FRAME_OUT stFrame;
        if (!camera.GetFrame(src_image, stFrame)) { usleep(10000); continue; }
        frame_num++;
        ret = inference_yolov8_model(&rknn_app_ctx, &src_image, &od_results);
        if (ret != 0) { free(src_image.virt_addr); continue; }
        PrintDetectionResult(frame_num, &od_results);
        static int aim_idx = -1;
        static struct timeval aim_tv = {0, 0};
        static int aim_count = 0;
        static uint16_t aim_gx = 32768, aim_gy = 32768;
        static uint16_t last_gx = 0, last_gy = 0;

        struct timeval now;
        gettimeofday(&now, NULL);
        double elapsed = (now.tv_sec - aim_tv.tv_sec) + (now.tv_usec - aim_tv.tv_usec) / 1000000.0;
        if (aim_idx == -2) { goto skip_aim; }
        if (galvo_fd < 0) { goto skip_aim; }

        if (aim_idx == -1) {
            // idle: wait for new detection
            if (od_results.count > 0) {
                aim_idx = 0;
                aim_count = od_results.count;
                aim_tv = now;
                auto* d = &od_results.results[0];
                int cx = (d->box.left + d->box.right) / 2;
                int cy = (d->box.top + d->box.bottom) / 2;
                pixel_to_dac(cx, cy, &aim_gx, &aim_gy);
                set_galvo_xy(galvo_fd, aim_gx, aim_gy);
                last_gx = aim_gx; last_gy = aim_gy;
                printf("  -> [%d/%d] %s DAC(%u,%u)\n", aim_idx+1, aim_count, coco_cls_to_name(d->cls_id), aim_gx, aim_gy);
            }
            goto skip_aim;
        }

        if (aim_gx != last_gx || aim_gy != last_gy) {
            set_galvo_xy(galvo_fd, aim_gx, aim_gy);
            last_gx = aim_gx; last_gy = aim_gy;
        }

        if (elapsed >= 2.0) {
            aim_idx++;
            if (aim_idx >= aim_count) {
                // all done: stay at last weed position, never move again
                printf("  -> All done, stay at DAC(%u,%u)\n", aim_gx, aim_gy);
                aim_idx = -2;
            } else {
                aim_tv = now;
                auto* d = &od_results.results[aim_idx];
                int cx = (d->box.left + d->box.right) / 2;
                int cy = (d->box.top + d->box.bottom) / 2;
                pixel_to_dac(cx, cy, &aim_gx, &aim_gy);
                set_galvo_xy(galvo_fd, aim_gx, aim_gy);
                last_gx = aim_gx; last_gy = aim_gy;
                printf("  -> [%d/%d] %s DAC(%u,%u)\n", aim_idx+1, aim_count, coco_cls_to_name(d->cls_id), aim_gx, aim_gy);
            }
        }
        
        skip_aim: ;
        free(src_image.virt_addr);
    }

out:
    if (galvo_fd >= 0) { set_galvo_xy(galvo_fd, 32768, 32768); galvo_spi_close(galvo_fd); }
    deinit_post_process();
    release_yolov8_model(&rknn_app_ctx);
    camera.Destroy();
    pthread_join(exit_thread, NULL);
    printf("Exit\n");
    return 0;
}
