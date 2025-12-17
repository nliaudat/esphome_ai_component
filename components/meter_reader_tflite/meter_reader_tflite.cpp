#include "meter_reader_tflite.h"
#include "esphome/core/application.h"

#include <esp_heap_caps.h>
#include <numeric>

#ifdef USE_WEB_SERVER
#include "esphome/components/esp32_camera_utils/preview_web_handler.h"
#endif

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

#ifdef SUPPORT_DOUBLE_BUFFERING
// Simple fixed-size object pools for InferenceJob and InferenceResult
using InferenceJob = MeterReaderTFLite::InferenceJob;
using InferenceResult = MeterReaderTFLite::InferenceResult;

static constexpr size_t INFERENCE_POOL_SIZE = 4;
static InferenceJob inference_job_pool[INFERENCE_POOL_SIZE];
static bool inference_job_used[INFERENCE_POOL_SIZE] = {false};
static InferenceResult inference_result_pool[INFERENCE_POOL_SIZE];
static bool inference_result_used[INFERENCE_POOL_SIZE] = {false};

static InferenceJob* allocate_inference_job() {
  for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
    if (!inference_job_used[i]) {
      inference_job_used[i] = true;
      inference_job_pool[i].frame = nullptr;
      inference_job_pool[i].crops.clear();
      inference_job_pool[i].start_time = 0;
      return &inference_job_pool[i];
    }
  }
  ESP_LOGW(TAG, "InferenceJob pool exhausted – allocating on heap");
  return new InferenceJob();
}

static void free_inference_job(InferenceJob* job) {
  // Release resources immediately to free up camera framebuffer
  job->frame.reset(); 
  job->crops.clear();

  for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
    if (job == &inference_job_pool[i]) {
      inference_job_used[i] = false;
      return;
    }
  }
  delete job;
}

static InferenceResult* allocate_inference_result() {
  for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
    if (!inference_result_used[i]) {
      inference_result_used[i] = true;
      inference_result_pool[i].inference_time = 0;
      inference_result_pool[i].success = false;
      inference_result_pool[i].readings.clear();
      inference_result_pool[i].probabilities.clear();
      return &inference_result_pool[i];
    }
  }
  ESP_LOGW(TAG, "InferenceResult pool exhausted – allocating on heap");
  return new InferenceResult();
}

static void free_inference_result(InferenceResult* res) {
  for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
    if (res == &inference_result_pool[i]) {
      inference_result_used[i] = false;
      return;
    }
  }
  delete res;
}
#endif




// Named constants for better readability (compile-time only)
static constexpr uint32_t MODEL_LOAD_DELAY_MS = 10000;

#ifdef DEBUG_METER_READER_TFLITE
#define METER_DURATION_START(name) uint32_t start_time = millis();
#define METER_DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)
#else
#define METER_DURATION_START(name)
#define METER_DURATION_END(name)
#endif

void MeterReaderTFLite::setup() {
    ESP_LOGI(TAG, "Setting up Meter Reader TFLite (Refactored)...");
    
    // 1. Initial Config
    if (camera_coord_.get_width() == 0) {
        ESP_LOGE(TAG, "Camera dimensions not set!");
        mark_failed(); return;
    }
    

    // 2. Load Model
    // Load model first to ensure we can retrieve input specifications for camera configuration.
    ESP_LOGI(TAG, "Model loading will begin in %u ms...", MODEL_LOAD_DELAY_MS);
    // Delayed to ensure WiFi logger captures these startup logs
    this->set_timeout(MODEL_LOAD_DELAY_MS, [this]() {
         ESP_LOGI(TAG, "Starting model loading...");
         esphome::App.feed_wdt(); // Feed before potentially long load
         if (!tflite_coord_.load_model()) {
             mark_failed(); return;
         }
         
         // After model loaded, we have input specs. Update CameraCoord.
         auto spec = tflite_coord_.get_model_spec();
         
         // Map input type: 
         // spec.input_type: 1=Float, 0=Uint8 (from TFLiteCoord)
         // camera/image_processor: 0=Float, 1=Uint8 (ImageProcessorInputType)
         int processor_input_type = (spec.input_type == 1) ? 0 : 1;
         
         camera_coord_.update_image_processor_config(
             spec.input_width, 
             spec.input_height, 
             spec.input_channels,
             processor_input_type,
             spec.normalize,
             spec.input_order
         );
         
         // Sync Esp32CameraUtils if present (for sensors)
         if (esp32_camera_utils_) {
              // Ensure it knows current dimensions
              esp32_camera_utils_->set_camera_image_format(
                  camera_coord_.get_width(),
                  camera_coord_.get_height(),
                  camera_coord_.get_format()
              );
              
              esp32_camera_utils::ImageProcessorConfig config;
              config.camera_width = camera_coord_.get_width();
              config.camera_height = camera_coord_.get_height();
              // Determine format based on model input channels (Coordinator logic)
              if (spec.input_channels == 1) {
                  config.pixel_format = "GRAYSCALE";
              } else {
                  config.pixel_format = camera_coord_.get_format();
              }
              config.model_width = spec.input_width;
              config.model_height = spec.input_height;
              config.model_channels = spec.input_channels;
              
              switch(static_cast<int>(rotation_)) {
                  case 90:  config.rotation = esp32_camera_utils::ROTATION_90;  break;
                  case 180: config.rotation = esp32_camera_utils::ROTATION_180; break;
                  case 270: config.rotation = esp32_camera_utils::ROTATION_270; break;
                  default:  config.rotation = esp32_camera_utils::ROTATION_0;   break;
              }
              
              config.input_type = static_cast<esp32_camera_utils::ImageProcessorInputType>(processor_input_type);
              config.normalize = spec.normalize;
              config.input_order = spec.input_order;
              
              esp32_camera_utils_->reinitialize_image_processor(config);
         }
         
          // Setup Web Server Preview
          #ifdef USE_WEB_SERVER
          #ifdef DEV_ENABLE_ROTATION
          if (web_server_) {
              web_server_->add_handler(new esphome::esp32_camera_utils::PreviewWebHandler([this]() {
                  return this->get_preview_image();
              }));
          }
          #endif
          #endif

          // Print debug info on success (legacy behavior)
          #ifdef DEBUG_METER_READER_TFLITE
          this->print_debug_info();
          #endif
          
          // Publish static memory stats
          #ifdef DEBUG_METER_READER_MEMORY
          if (tensor_arena_size_sensor_) {
              // We need a getter for requested size in TFLiteCoord
              tensor_arena_size_sensor_->publish_state(tflite_coord_.get_tensor_arena_size()); 
          }
          #endif
     });
    
    // 3. Setup Validation
    ValueValidator::ValidationConfig val_conf;
    val_conf.allow_negative_rates = allow_negative_rates_;
    val_conf.max_absolute_diff = max_absolute_diff_;
    val_conf.high_confidence_threshold = high_confidence_threshold_;
    output_validator_.set_config(val_conf);
    output_validator_.setup();
    
    ESP_LOGI(TAG, "Output validation configured - AllowNegativeRates: %s, MaxAbsoluteDiff: %d, HighConfThreshold: %.2f",
             val_conf.allow_negative_rates ? "YES" : "NO", 
             val_conf.max_absolute_diff,
             val_conf.high_confidence_threshold);
    
    // 4. Setup Camera Callback
    // Register callback on the global camera instance.
    
    // 5. Setup Flashlight Coordinator Callback
    flashlight_coord_.set_request_frame_callback([this](){
        this->frame_requested_ = true;
        this->last_request_time_ = millis();
        ESP_LOGD(TAG, "Frame requested via coordinator callback");
    });
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
    // 6. Setup Double Buffering Pipeline
    ESP_LOGI(TAG, "Initializing Double Buffering (Dual Core mode)...");
    input_queue_ = xQueueCreate(1, sizeof(InferenceJob*)); // Pointer depth 1 (Backpressure)
    output_queue_ = xQueueCreate(2, sizeof(InferenceResult*)); // Pointer depth 2
    
    if (!input_queue_ || !output_queue_) {
        ESP_LOGE(TAG, "Failed to create queues!");
        mark_failed(); return;
    }
    
    BaseType_t res = xTaskCreatePinnedToCore(
        MeterReaderTFLite::inference_task, 
        "inference_task", 
        8192, // Stack size
        this, // Pass this instance
        1,    // Priority (Low)
        &inference_task_handle_, 
        0     // Pin to Core 0 (Main Loop is usually Core 1)
    );
    
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create inference task!");
        mark_failed(); return;
    }
    ESP_LOGI(TAG, "Double Buffering active on Core 0");
    #endif

    // Force Camera to Grayscale if using GRAY model (Workaround for YAML limit)
    // Only if user requested GRAYSCALE in config (we can't easily check the template sub here, 
    // but we know we just failed to set it in YAML).
    // Let's check the global pixel format config if feasible, or just force it if the model is V4 GRAY.
}

void MeterReaderTFLite::set_camera(camera::Camera *camera) { // -> CameraCoord callback
    camera_coord_.set_camera((esp32_camera::ESP32Camera*)camera);
    // Register listener
    ESP_LOGD(TAG, "Registering CameraListener for MeterReaderTFLite");
    camera->add_listener(this); 
}

void MeterReaderTFLite::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
    if (frame_requested_.load() && !processing_frame_.load()) {
        pending_frame_ = image;
        frame_available_.store(true);
        frame_requested_.store(false);
    }
}


void MeterReaderTFLite::update() {
    ESP_LOGD(TAG, "Update triggered (Interval cycle)");
    if (crop_zone_handler_.has_global_zones_changed()) {
        crop_zone_handler_.apply_global_zones();
    }
    // Trigger updates for external camera utils sensors if available
    if (esp32_camera_utils_) {
        esp32_camera_utils_->update_memory_sensors();
    }
    
    // 2. Crop Zones Processing
    if (pause_processing_) {
        ESP_LOGD(TAG, "Processing paused, skipping update");
        return;
    }

    // Calibration Cycle
    if (is_calibrating()) {
        update_calibration();
        return; // Skip normal processing during calibration
    }
    
    // The flashlight coordinator returns true if it is handling the cycle (scheduling or waiting)
    bool busy = flashlight_coord_.update_scheduling();
    
    if (!busy) {
        // Normal cycle
         if (!frame_available_ && !frame_requested_) {
             frame_requested_ = true;
             last_request_time_ = millis();
             ESP_LOGD(TAG, "Requesting frame (Continuous/No-Flash)");
         } else if (frame_available_) {
             process_available_frame();
         }
    }
}

void MeterReaderTFLite::start_flash_calibration() {
    if (is_calibrating()) return;
    
    ESP_LOGI(TAG, "Starting Flash Calibration...");
    calibration_.state = FlashCalibrationHandler::CALIBRATING_PRE;
    calibration_.current_pre = calibration_.start_pre;
    calibration_.current_post = calibration_.start_post; // Init
    calibration_.baseline_confidence = 0.0f; // Will be set by first run
    
    // Override current settings
    flashlight_coord_.set_timing(calibration_.current_pre, calibration_.current_post);
    
    if (inference_logs_) inference_logs_->publish_state("Starting Calibration: Phase 1 (Pre-Time)");
    
    // Trigger first step
    force_flash_inference();
}

void MeterReaderTFLite::update_calibration() {
    // This function is called every loop/update, but we mainly react to inference completion
    // The inference completion logic (process_full_image) needs to know we are calibrating.
    // Actually, force_flash_inference sets up a oneshot. 
    // We need to wait for the result.
    // The result is processed in process_full_image. We should capture the result there.
}

void MeterReaderTFLite::loop() {
    if (this->is_failed()) return;

    // Watchdog: If frame requested but not arrived, reset state
    if (frame_requested_ && (millis() - last_request_time_ > frame_request_timeout_ms_)) {
        ESP_LOGW(TAG, "Frame request timed out (%u ms)! Resetting state.", frame_request_timeout_ms_);
        frame_requested_ = false;
        // Check if we need to force reset camera or just continue
    }

    if (frame_available_ && !processing_frame_) {
        process_available_frame();
    }
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
    // Check for async results
    InferenceResult* res_ptr = nullptr;
    if (output_queue_ && xQueueReceive(output_queue_, &res_ptr, 0) == pdTRUE) {
        if (res_ptr) {
            // Reconstruct logic from process_full_image's tail
            ESP_LOGD(TAG, "Async inference finished in %lu ms (Total: %lu ms)", 
                     res_ptr->inference_time, millis() - res_ptr->total_start_time);
            
            // Reconstruct combined reading
            float final_val = combine_readings(res_ptr->readings);
            
            // Validate
            // Use Average Confidence to match Single Core logic
            float avg_conf = 0.0f;
            if (!res_ptr->probabilities.empty()) {
                 float sum = 0.0f;
                 for (float c : res_ptr->probabilities) sum += c;
                 avg_conf = sum / res_ptr->probabilities.size();
            }
            
            if (value_sensor_) {
                float validated_val = 0.0f;
                bool valid = validate_and_update_reading(final_val, avg_conf, validated_val);
                
                // Publish (matches process_full_image logic)
                if (valid && avg_conf >= confidence_threshold_) {
                     ESP_LOGI(TAG, "Result: VALID (Raw: %.0f, Conf: %.2f)", final_val, avg_conf);
                     value_sensor_->publish_state(validated_val);
                     if (confidence_sensor_) {
                         confidence_sensor_->publish_state(avg_conf * 100.0f);
                     }
                } else {
                     ESP_LOGI(TAG, "Result: INVALID (Raw: %.0f, Conf: %.2f)", final_val, avg_conf);
                }
            }
            
            // Cleanup
            free_inference_result(res_ptr);
            
            // Request next frame if configured?
            // Original logic: loop requests if continuous? 
            // set_timeout logic was used in some versions.
            // Here 'update()' triggers 'take_picture'.
        }
        processing_frame_ = false; // Release lock
    }
    #endif
}

void MeterReaderTFLite::process_available_frame() {
    processing_frame_ = true;
    std::shared_ptr<camera::CameraImage> frame = pending_frame_;
    pending_frame_.reset();
    frame_available_ = false;
    
    if (frame) {
        process_full_image(frame);
    }
    processing_frame_ = false;
}

void MeterReaderTFLite::process_full_image(std::shared_ptr<camera::CameraImage> frame) {
    if (pause_processing_) return;

    METER_DURATION_START("Total Processing");

    if (!tflite_coord_.is_model_loaded()) {
        ESP_LOGW(TAG, "Skipping frame - Model not loaded yet");
        return;
    }

    
    // Preview Logic (Rotation)
    #ifdef DEV_ENABLE_ROTATION
    if (generate_preview_ || request_preview_) {
         // Create rotated preview via ImageProcessor
         using namespace esphome::esp32_camera_utils;
         
         // Using dimensions from camera coord
         auto preview = ImageProcessor::generate_rotated_preview(
             frame, rotation_, camera_coord_.get_width(), camera_coord_.get_height());
             
         if (preview) {
             // Cast to RotatedPreviewImage to access width/height
             auto rotated_preview = std::static_pointer_cast<RotatedPreviewImage>(preview);

             // Draw Crop Zones if enabled
             if (show_crop_areas_) {
                 auto zones = crop_zone_handler_.get_zones();
                 uint8_t* buf = preview->get_data_buffer();
                 int w = rotated_preview->get_width();
                 int h = rotated_preview->get_height();
                 // Assuming Preview is RGB565 or RGB888. DrawingUtils handles both?
                 // DrawingUtils signatures take 'channels'.
                 // We need to know the format of the preview.
                 // ImageProcessor::generate_rotated_preview usually returns RGB565 for display.
                 // Let's assume RGB565 (2 bytes) for now as that's standard for ESPHome Camera.
                 
                 // However, we should check pixel format if possible, but CameraImage usually abstracts it.
                 // Safe bet: RGB565 is 2 channels. 
                 
                 // Color: Light Green (0x9FD3 or similar). 0x07E0 is pure green.
                 uint16_t color = 0x07E0; 
                 
                 for (const auto& z : zones) {
                     // Zones are in the same coordinate space as the preview (rotated)
                     esphome::esp32_camera_utils::DrawingUtils::draw_rectangle(
                         buf, z.x1, z.y1, z.x2 - z.x1, z.y2 - z.y1,
                         w, h, 2, color
                     );
                 }
             }
             update_preview_image(preview);
         }
         
         if (request_preview_) {
             request_preview_ = false;
             return; // Skip inference
         }
    }
    #endif

    // Inference
    auto zones = crop_zone_handler_.get_zones();
    ESP_LOGI(TAG, "Processing Image: Found %d crop zones", zones.size());
    
    // Process frame -> buffers
    esphome::App.feed_wdt();
    auto processed_buffers = camera_coord_.process_frame(frame, zones);
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
    // Async Path
    if (processed_buffers.empty()) {
        processing_frame_ = false;
        return;
    }
    
    InferenceJob* job = allocate_inference_job();
    job->frame = frame; // Keep alive
    job->crops = std::move(processed_buffers);
    // Use the start_time from METER_DURATION_START (defined at start of function)
    job->start_time = start_time; 
    
    ESP_LOGD(TAG, "Preprocessing took %u ms", millis() - start_time);

    if (xQueueSend(input_queue_, &job, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Inference Queue Full - Dropping Frame");
        free_inference_job(job);
        processing_frame_ = false;
    }
    // Return immediately, loop() will handle result
    return;
    #endif
    
    // Buffers -> Inference (Synchronous Fallback)
    
    // Buffers -> Inference
    
    // Capture Peak Memory State *during* processing (buffers allocated)
    #ifdef DEBUG_METER_READER_MEMORY
    if (debug_memory_enabled_) {
        if (process_free_heap_sensor_) process_free_heap_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (process_free_psram_sensor_) process_free_psram_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    #endif

    auto results = tflite_coord_.run_inference(processed_buffers);
    
    #ifdef DEBUG_METER_READER_MEMORY
    if (debug_memory_enabled_ && tensor_arena_used_sensor_) {
          tensor_arena_used_sensor_->publish_state(tflite_coord_.get_arena_used_bytes());
    }
    #endif

    
    // Collect readings
    std::vector<float> readings, confidences;
    int digit_index = 0;
    for (const auto& res : results) {
        if (res.success) {
            ESP_LOGD(TAG, "Digit %d: %.0f (%.2f)", digit_index, res.value, res.confidence);
            readings.push_back(res.value);
            confidences.push_back(res.confidence);
        } else {
             ESP_LOGW(TAG, "Digit %d: Failed to infer", digit_index);
        }
        digit_index++;
    }
    
    if (!readings.empty()) {
        // Use helper to combine readings and log details (matches legacy behavior)
        float final_val = combine_readings(readings);

        float avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / confidences.size();
        
        float validated_val = final_val;
        bool valid = validate_and_update_reading(final_val, avg_conf, validated_val);

        if (inference_logs_) {
             // Publish to inference logs text sensor
             char inference_log[150];
             snprintf(inference_log, sizeof(inference_log),
                      "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)",
                      final_val, validated_val, valid ? "yes" : "no",
                      avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             inference_logs_->publish_state(inference_log);
        }

        if (valid && avg_conf >= confidence_threshold_) {
             // Removed checking of inference_log char buffer availability to match legacy cleanly
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             
             if (value_sensor_) value_sensor_->publish_state(validated_val);
             if (confidence_sensor_) confidence_sensor_->publish_state(avg_conf * 100.0f);
             
             ESP_LOGI(TAG, "Reading published - valid and confidence threshold met");
        } else {
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             ESP_LOGW(TAG, "Reading NOT published - %s", 
                     !valid ? "validation failed" : "confidence below threshold");
        }

        // Calibration Logic Hook
        if (is_calibrating()) {
             char log_msg[100];
             bool next_step = false;
             
             // 1. Establish Baseline (First run of Pre-Phase)
             if (calibration_.baseline_confidence == 0.0f) {
                 calibration_.baseline_confidence = avg_conf;
                 calibration_.best_pre = calibration_.current_pre;
                 calibration_.best_post = calibration_.current_post;
                 calibration_.best_confidence = avg_conf; 
                 snprintf(log_msg, sizeof(log_msg), "Baseline Conf: %.1f%%. Testing Pre: %u", avg_conf * 100.0f, calibration_.current_pre);
                 next_step = true;
             } else {
                 // Check if worse
                 if (avg_conf < (calibration_.baseline_confidence - 0.05f)) { // 5% drop threshold
                     snprintf(log_msg, sizeof(log_msg), "Conf dropped to %.1f%% (Baseline %.1f%%). Step Failed.", avg_conf * 100.0f, calibration_.baseline_confidence * 100.0f);
                     // Dropped too much, stop this phase or revert
                     // For Pre Phase: Stop, use last good
                     if (calibration_.state == FlashCalibrationHandler::CALIBRATING_PRE) {
                         calibration_.state = FlashCalibrationHandler::CALIBRATING_POST;
                         calibration_.current_pre = calibration_.best_pre; // Revert
                         calibration_.current_post = calibration_.start_post; // Start Post phase
                         snprintf(log_msg, sizeof(log_msg), "Pre-Phase Done. Best: %ums. Starting Post-Phase.", calibration_.best_pre);
                     } else if (calibration_.state == FlashCalibrationHandler::CALIBRATING_POST) {
                         calibration_.state = FlashCalibrationHandler::FINISHED;
                         calibration_.current_post = calibration_.best_post; // Revert
                         snprintf(log_msg, sizeof(log_msg), "Calibration Done! Best: Pre=%ums, Post=%ums", calibration_.best_pre, calibration_.best_post);
                         if (inference_logs_) inference_logs_->publish_state(log_msg);
                         // Don't trigger next step
                     }
                 } else {
                     // Good result, store as best and continue
                     calibration_.best_confidence = avg_conf; // Optionally update baseline? No, keep original baseline to prevent drift? Or update? Keep original generic baseline.
                     if (calibration_.state == FlashCalibrationHandler::CALIBRATING_PRE) {
                         calibration_.best_pre = calibration_.current_pre;
                         snprintf(log_msg, sizeof(log_msg), "Pre-Time %ums OK (Wait 1s...)", calibration_.current_pre);
                         next_step = true;
                     } else {
                         calibration_.best_post = calibration_.current_post;
                         snprintf(log_msg, sizeof(log_msg), "Post-Time %ums OK (Wait 1s...)", calibration_.current_post);
                         next_step = true;
                     }
                 }
             }
             ESP_LOGI(TAG, "%s", log_msg);
             if (inference_logs_) inference_logs_->publish_state(log_msg);
             
             if (calibration_.state != FlashCalibrationHandler::FINISHED) { // && next_step logic
                  // Advance parameters
                  if (next_step) {
                      if (calibration_.state == FlashCalibrationHandler::CALIBRATING_PRE) {
                          if (calibration_.current_pre > (calibration_.end_pre + calibration_.step_pre)) {
                              calibration_.current_pre -= calibration_.step_pre;
                          } else {
                              // Reached end of range
                              calibration_.state = FlashCalibrationHandler::CALIBRATING_POST;
                              calibration_.current_pre = calibration_.best_pre;
                              calibration_.current_post = calibration_.start_post;
                          }
                      } else {
                          if (calibration_.current_post > (calibration_.end_post + calibration_.step_post)) {
                              calibration_.current_post -= calibration_.step_post;
                          } else {
                              calibration_.state = FlashCalibrationHandler::FINISHED;
                          }
                      }
                      
                      if (calibration_.state != FlashCalibrationHandler::FINISHED) {
                           // Delay slightly before next flash?
                           flashlight_coord_.set_timing(calibration_.current_pre, calibration_.current_post);
                            // Simple delay or timeout?
                           this->set_timeout(1000, [this](){
                                force_flash_inference();
                           });
                      } else {
                          // Finished at end of range
                          char fin[100];
                          snprintf(fin, sizeof(fin), "Done. Optimal: Pre=%ums, Post=%ums", calibration_.best_pre, calibration_.best_post);
                          if (inference_logs_) inference_logs_->publish_state(fin);
                      }
                  } else {
                       // Failed case logic above handles state transitions
                       if (calibration_.state != FlashCalibrationHandler::FINISHED) {
                             flashlight_coord_.set_timing(calibration_.current_pre, calibration_.current_post);
                             this->set_timeout(1000, [this](){
                                force_flash_inference();
                             });
                       }
                  }
             }
        }
    }
    
    METER_DURATION_END("Total Processing");
}

void MeterReaderTFLite::set_crop_zones(const std::string &zones_json) {
    crop_zone_handler_.update_zones(zones_json);
}

// Config Delegators
void MeterReaderTFLite::set_tensor_arena_size(size_t size) { tflite_coord_.set_tensor_arena_size(size); } 
void MeterReaderTFLite::set_model_config(const std::string& type) { tflite_coord_.set_model_type(type); }
void MeterReaderTFLite::set_model(const uint8_t *model, size_t length) { tflite_coord_.set_model(model, length); }

void MeterReaderTFLite::set_camera_image_format(int w, int h, const std::string &fmt) {
    camera_coord_.set_config(w, h, fmt);
    // Update ImageProcessor config if model is loaded 
    if (tflite_coord_.is_model_loaded()) {
         auto spec = tflite_coord_.get_model_spec();
         camera_coord_.update_image_processor_config(
             spec.input_width, spec.input_height, spec.input_channels,
             spec.input_type, spec.normalize, spec.input_order);
    }
}

void MeterReaderTFLite::set_flash_light(light::LightState* light) {
    flashlight_coord_.setup(this, light, nullptr);
}
void MeterReaderTFLite::set_flash_controller(flash_light_controller::FlashLightController* c) {
    flashlight_coord_.setup(this, nullptr, c);
}
void MeterReaderTFLite::set_flash_pre_time(uint32_t ms) { flashlight_coord_.set_timing(ms, 2000); } // simplified
void MeterReaderTFLite::set_flash_post_time(uint32_t ms) { flashlight_coord_.set_timing(5000, ms); }

// Preview
#ifdef DEV_ENABLE_ROTATION
void MeterReaderTFLite::take_preview_image() { capture_preview(); }
void MeterReaderTFLite::capture_preview() {
    request_preview_ = true;
    flashlight_coord_.capture_preview_sequence([this](){
        frame_requested_ = true;
        last_request_time_ = millis();
    });
}
std::shared_ptr<camera::CameraImage> MeterReaderTFLite::get_preview_image() {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    return last_preview_image_;
}
void MeterReaderTFLite::update_preview_image(std::shared_ptr<camera::CameraImage> image) {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    last_preview_image_ = image;
}
#endif

// Logic Helpers
float MeterReaderTFLite::combine_readings(const std::vector<float>& readings) {
    std::string digit_string;
    
    ESP_LOGI(TAG, "Processing %d readings:", readings.size());
    
    // Convert each reading to integer digit and handle wrap-around
    for (size_t i = 0; i < readings.size(); i++) {
        int digit = static_cast<int>(round(readings[i]));
        
        // Handle wrap-around for original models (like Python script)
        if (digit == 10) {
            digit = 0;
            ESP_LOGD(TAG, "Zone %d: Raw=%.1f -> Rounded=10 -> Wrapped to 0", 
                    i + 1, readings[i]);
        } else {
            ESP_LOGD(TAG, "Zone %d: Raw=%.1f -> Rounded=%d", 
                    i + 1, readings[i], digit);
        }
        
        digit_string += std::to_string(digit);
    }
    
    ESP_LOGI(TAG, "Concatenated digit string: %s", digit_string.c_str());
    
    std::string readings_str;
    for (const auto& reading : readings) {
      if (!readings_str.empty()) {
        readings_str += ", ";
      }
      char buffer[16];
      snprintf(buffer, sizeof(buffer), "%.1f", reading);
      readings_str += buffer;
    }
    ESP_LOGD(TAG, "Raw readings: [%s]", readings_str.c_str());
    
    float combined_value = 0.0f;
    // Guaranteed to be numeric string from logic above
    combined_value = std::stof(digit_string);
    
    ESP_LOGI(TAG, "Final combined value: %.0f", combined_value);
    return combined_value;
}

bool MeterReaderTFLite::validate_and_update_reading(float raw, float conf, float& val) {
    int ival = static_cast<int>(raw);
    int oval = ival;
    bool valid = output_validator_.validate_reading(ival, conf, oval);
    val = static_cast<float>(oval);
    return valid;
}

// Window Control
void MeterReaderTFLite::set_camera_window_offset_x(int x) { 
    camera_coord_.set_window_config(x, -1, -1, -1); 
    if (window_active_) camera_coord_.apply_window();
}
void MeterReaderTFLite::set_camera_window_offset_y(int y) { 
    camera_coord_.set_window_config(-1, y, -1, -1); 
    if (window_active_) camera_coord_.apply_window();
}
void MeterReaderTFLite::set_camera_window_width(int w) { 
    camera_coord_.set_window_config(-1, -1, w, -1); 
    if (window_active_) camera_coord_.apply_window();
}
void MeterReaderTFLite::set_camera_window_height(int h) { 
    camera_coord_.set_window_config(-1, -1, -1, h); 
    if (window_active_) camera_coord_.apply_window();
}
void MeterReaderTFLite::set_camera_window_configured(bool c) { 
    window_active_ = c;
    if (c) {
        // Trigger window application logic now that configuration is complete.
        if (camera_coord_.apply_window()) {
             ESP_LOGD(TAG, "Window configuration applied successfully.");
        } else {
             ESP_LOGW(TAG, "Failed to apply window configuration.");
             window_active_ = false; // Revert if failed
        }
    } else {
        reset_camera_window();
    }
}

bool MeterReaderTFLite::reset_camera_window() {
    bool success = camera_coord_.reset_window();
    // Only clear active flag if reset successful (or forced?) 
    // Usually reset should imply back to full frame
    if (success) {
         window_active_ = false;
         // Note: We do NOT clear the stored config values in CameraCoord, so user can re-enable later.
    }
    return success;
}

bool MeterReaderTFLite::set_camera_window(int offset_x, int offset_y, int width, int height) {
     bool success = camera_coord_.set_window(offset_x, offset_y, width, height);
     if (success) {
         window_active_ = true;
         // Also update the stored config so partial setters work later
         camera_coord_.set_window_config(offset_x, offset_y, width, height);
     }
     return success;
}

// Destructor
MeterReaderTFLite::~MeterReaderTFLite() {
#ifdef SUPPORT_DOUBLE_BUFFERING
    // Clean up FreeRTOS resources
    if (inference_task_handle_) {
        vTaskDelete(inference_task_handle_);
        inference_task_handle_ = nullptr;
    }
    if (input_queue_) {
        vQueueDelete(input_queue_);
        input_queue_ = nullptr;
    }
    if (output_queue_) {
        vQueueDelete(output_queue_);
        output_queue_ = nullptr;
    }
#endif
} 

void MeterReaderTFLite::set_update_interval(uint32_t ms) {
    ESP_LOGI(TAG, "Setting update interval: %u ms", ms);
    PollingComponent::set_update_interval(ms);
}

void MeterReaderTFLite::force_flash_inference() {
    flashlight_coord_.force_inference([this](){ 
        frame_requested_ = true; 
        last_request_time_ = millis();
    });
}

// Debug Handlers
#ifdef DEBUG_METER_READER_TFLITE
void MeterReaderTFLite::set_debug_image(const uint8_t* data, size_t size) {
    // Hardcoded dimensions for now or need to know context
    debug_coord_.set_debug_image(data, size, camera_coord_.get_width(), camera_coord_.get_height());
}
void MeterReaderTFLite::test_with_debug_image() {
    debug_coord_.run_debug_tests(tflite_coord_);
}
void MeterReaderTFLite::set_debug_mode(bool m) {
    debug_coord_.set_debug_mode(m);
}
void MeterReaderTFLite::debug_test_with_pattern() {
    debug_coord_.test_with_pattern(tflite_coord_);
}
#endif

void MeterReaderTFLite::print_debug_info() {
    debug_coord_.print_info(tflite_coord_, camera_coord_.get_width(), camera_coord_.get_height(), camera_coord_.get_format());
}

void MeterReaderTFLite::dump_config() {
  ESP_LOGCONFIG(TAG, "Meter Reader TFLite:");
  ESP_LOGCONFIG(TAG, "  Confidence Threshold: %.2f", this->confidence_threshold_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->get_update_interval());
  
  #ifdef SUPPORT_DOUBLE_BUFFERING
  ESP_LOGCONFIG(TAG, "  Pipeline: Double Buffering (Dual Core) ENABLED");
  #else
  ESP_LOGCONFIG(TAG, "  Pipeline: Single Core (Sequential)");
  #endif

  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Component FAILED to setup");
  }
}

} // namespace meter_reader_tflite
} // namespace esphome

#ifdef SUPPORT_DOUBLE_BUFFERING
using namespace esphome::meter_reader_tflite;

void MeterReaderTFLite::inference_task(void *arg) {
    MeterReaderTFLite* self = (MeterReaderTFLite*)arg;
    InferenceJob* job = nullptr;
    
    while (true) {
        if (xQueueReceive(self->input_queue_, &job, portMAX_DELAY) == pdTRUE) {
            if (!job) continue;
            
            uint32_t start = millis();
            
            // Invoke TFLite (Thread-Safe because only this task calls it)
            // Note: queues are thread safe.
            auto tflite_results = self->tflite_coord_.run_inference(job->crops);
            
            InferenceResult* res = allocate_inference_result();
            res->inference_time = millis() - start;
            res->total_start_time = job->start_time;
            res->success = true;
            
            for (auto& r : tflite_results) {
                res->readings.push_back(r.value);
                res->probabilities.push_back(r.confidence);
            }
            
            // Clean up job inputs
            free_inference_job(job);
            
            // Send back
            if (xQueueSend(self->output_queue_, &res, 100 / portTICK_PERIOD_MS) != pdTRUE) {
                // Main loop stuck? Drop result.
                free_inference_result(res);
            }
        }
    }
}
#endif
