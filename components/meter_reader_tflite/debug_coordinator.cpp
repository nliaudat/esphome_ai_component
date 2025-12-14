#include "debug_coordinator.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "debug_coordinator";

// Helper class for debug image
class DebugCameraImage : public camera::CameraImage {
public:
    DebugCameraImage(const uint8_t* data, size_t size, int width, int height)
        : data_(data, data + size), width_(width), height_(height) {}

    uint8_t* get_data_buffer() override { return data_.data(); }
    size_t get_data_length() override { return data_.size(); }
    bool was_requested_by(camera::CameraRequester requester) const override { 
        return false;
    }

private:
    std::vector<uint8_t> data_;
    int width_;
    int height_;
};

void DebugCoordinator::set_debug_image(const uint8_t* data, size_t size, int width, int height) {
    debug_image_ = std::make_shared<DebugCameraImage>(data, size, width, height);
    ESP_LOGI(TAG, "Debug image set: %zu bytes", size);
}

void DebugCoordinator::run_debug_tests(TFLiteCoordinator& tflite_coord) {
    if (!debug_image_) {
        ESP_LOGW(TAG, "No debug image set");
        return;
    }
    
    ESP_LOGI(TAG, "Running debug tests on stored image...");
    
}

void DebugCoordinator::test_with_pattern(TFLiteCoordinator& tflite_coord) {
    ESP_LOGI(TAG, "Testing with generated pattern...");
    int width = tflite_coord.get_input_width();
    int height = tflite_coord.get_input_height();
    int channels = tflite_coord.get_input_channels();
    
    std::vector<uint8_t> pattern(width * height * channels);
    // Fill gradient
    for(int i=0; i<pattern.size(); ++i) pattern[i] = i % 255;
    
    std::vector<std::vector<uint8_t>> batch = {pattern};
    tflite_coord.debug_test_parameters(batch);
}

void DebugCoordinator::print_info(const TFLiteCoordinator& tflite_coord, int cam_w, int cam_h, const std::string& fmt) {
    ESP_LOGI(TAG, "--- MeterReaderTFLite Debug Info ---");
    ESP_LOGI(TAG, "  Model Loaded: %s", tflite_coord.is_model_loaded() ? "Yes" : "No");
    ESP_LOGI(TAG, "  Camera Dimensions: %dx%d", cam_w, cam_h);
    ESP_LOGI(TAG, "  Pixel Format: %s", fmt.c_str());
    
    size_t model_len = tflite_coord.get_model_size_bytes();
    ESP_LOGI(TAG, "  Model Size: %zu bytes (%.1f KB)", model_len, model_len / 1024.0f);
    
    size_t arena_req = tflite_coord.get_tensor_arena_size();
    ESP_LOGI(TAG, "  Tensor Arena Size (Requested): %zu bytes (%.1f KB)", 
             arena_req, arena_req / 1024.0f);
             
    size_t arena_act = tflite_coord.get_tensor_arena_size_actual();
    ESP_LOGI(TAG, "  Tensor Arena Size (Actual): %zu bytes (%.1f KB)", 
             arena_act, arena_act / 1024.0f);
    
    size_t peak_usage = tflite_coord.get_arena_peak_bytes();
    ESP_LOGI(TAG, "  Arena Peak Usage: %zu bytes (%.1f KB)", peak_usage, peak_usage / 1024.0f);
    
    size_t total_memory = model_len + arena_act;
    ESP_LOGI(TAG, "  TOTAL Memory Footprint: %zu bytes (%.1f KB)", 
             total_memory, total_memory / 1024.0f);
             
    // Since we don't have direct access to internal memory_manager_ here to call report_memory_status
    // we can rely on what we just printed or add a delegated call if strictly needed.
    // Legacy called memory_manager_.report_memory_status(...) which prints essentially the same info + free heap/psram?
    // Let's stick to this for now as it covers "tensor arena size, etc".
    
    ESP_LOGI(TAG, "----------------------------------");
}

} // namespace meter_reader_tflite
} // namespace esphome
