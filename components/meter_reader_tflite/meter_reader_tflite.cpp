#include "meter_reader_tflite.h"
#include "esphome/core/application.h"


#include <esp_heap_caps.h>

#include <numeric>

#ifdef USE_WEB_SERVER
#include "esphome/components/esp32_camera_utils/preview_web_handler.h"
#endif

// Add missing include for DrawingUtils
#if !defined(USE_HOST) && defined(USE_CAMERA_DRAWING)
#include "esphome/components/esp32_camera_utils/drawing_utils.h"
#endif

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

#ifdef SUPPORT_DOUBLE_BUFFERING
// Dynamic object pools for InferenceJob and InferenceResult
using InferenceJob = MeterReaderTFLite::InferenceJob;
using InferenceResult = MeterReaderTFLite::InferenceResult;

// Pool size determined at runtime based on available memory
static size_t INFERENCE_POOL_SIZE = 4; // Will be adjusted in setup
static constexpr size_t MIN_POOL_SIZE = 2;
static constexpr size_t MAX_POOL_SIZE = 8;

// Dynamic pool arrays (allocated at setup)
static std::unique_ptr<InferenceJob[]> inference_job_pool;
static std::unique_ptr<bool[]> inference_job_used;
static std::unique_ptr<InferenceResult[]> inference_result_pool;
static std::unique_ptr<bool[]> inference_result_used;

// Pool efficiency statistics
static std::atomic<uint32_t> pool_job_hits{0};
static std::atomic<uint32_t> pool_job_misses{0};
static std::atomic<uint32_t> pool_result_hits{0};
static std::atomic<uint32_t> pool_result_misses{0};

// Calculate optimal pool size based on available RAM
// Conservative for ESP32, more aggressive for ESP32-S3 with PSRAM
static size_t calculate_optimal_pool_size() {
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t total_free = free_psram + free_internal;
  
  ESP_LOGD(TAG, "Memory available - PSRAM: %zu KB, Internal: %zu KB", 
           free_psram / 1024, free_internal / 1024);
  
  // Each pool entry uses ~1-2KB (frame shared_ptr + crops vector + metadata)
  // Strategy: Very conservative on ESP32, moderate on ESP32-S3
  
  if (free_psram > 4 * 1024 * 1024) {
    // ESP32-S3 with >4MB PSRAM - can afford larger pool
    ESP_LOGI(TAG, "High PSRAM detected, using MAX pool size");
    return MAX_POOL_SIZE;
  } else if (free_psram > 2 * 1024 * 1024) {
    // ESP32-S3 with >2MB PSRAM - moderate pool
    ESP_LOGI(TAG, "Moderate PSRAM detected, using 6-entry pool");
    return 6;
  } else if (free_internal > 200 * 1024) {
    // ESP32 with good internal RAM - standard pool
    ESP_LOGI(TAG, "Sufficient internal RAM, using 4-entry pool");
    return 4;
  } else if (free_internal > 150 * 1024) {
    // ESP32 with limited RAM - conservative pool
    ESP_LOGW(TAG, "Limited internal RAM, using 3-entry pool");
    return 3;
  } else {
    // Very limited RAM - minimal pool
    ESP_LOGW(TAG, "Low memory detected, using MIN pool size");
    return MIN_POOL_SIZE;
  }
}

static InferenceJob* allocate_inference_job() {
  // ESP_LOGD(TAG, "Allocating InferenceJob..."); 
  for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
    if (!inference_job_used[i]) {
      inference_job_used[i] = true;
      inference_job_pool[i].frame = nullptr;
      inference_job_pool[i].crops.clear();
      inference_job_pool[i].start_time = 0;
      pool_job_hits++;
      // ESP_LOGD(TAG, "Allocated from pool (index %d)", i);
      return &inference_job_pool[i];
    }
  }
  
  pool_job_misses++;
  uint32_t total = pool_job_hits + pool_job_misses;
  if (total > 0 && (total % 100 == 0)) {  // Log every 100 allocations
    ESP_LOGW(TAG, "InferenceJob pool exhausted (efficiency: %.1f%%) – allocating on heap",
             100.0f * pool_job_hits / total);
  }
  ESP_LOGI(TAG, "Pool exhausted. Allocating new InferenceJob on heap.");
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
      // Force memory release by swapping with empty temporary, 
      // ensuring no hidden capacity grows indefinitely in the pool.
      std::vector<float>().swap(inference_result_pool[i].readings);
      std::vector<float>().swap(inference_result_pool[i].probabilities);
      pool_result_hits++;
      return &inference_result_pool[i];
    }
  }
  
  pool_result_misses++;
  uint32_t total = pool_result_hits + pool_result_misses;
  if (total > 0 && (total % 100 == 0)) {  // Log every 100 allocations
    ESP_LOGW(TAG, "InferenceResult pool exhausted (efficiency: %.1f%%) – allocating on heap",
             100.0f * pool_result_hits / total);
  }
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

static constexpr uint32_t CALIBRATION_START_PRE_MS = 500;
static constexpr uint32_t CALIBRATION_END_PRE_MS = 100;
static constexpr uint32_t CALIBRATION_STEP_PRE_MS = 100;

static constexpr uint32_t CALIBRATION_START_POST_MS = 500;
static constexpr uint32_t CALIBRATION_END_POST_MS = 100;
static constexpr uint32_t CALIBRATION_STEP_POST_MS = 100;

#define METER_DURATION_START(name) uint32_t start_time = millis();
#define METER_DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)

void MeterReaderTFLite::setup() {
    ESP_LOGI(TAG, "Setting up Meter Reader TFLite (Refactored)...");
    
    // 1. Initial Config
    if (camera_coord_.get_width() == 0) {
        ESP_LOGE(TAG, "Camera dimensions not set!");
        if (main_logs_) main_logs_->publish_state("CRITICAL: Camera dimensions not set!");
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
         
         // [MOVED] Sync Rotation FIRST
         if (esp32_camera_utils_) {
              // Sync rotation from Utils (Centralized Configuration)
              this->rotation_ = esp32_camera_utils_->get_rotation();
              ESP_LOGI(TAG, "Synced rotation from esp32_camera_utils: %.1f", this->rotation_);
              
              // Pass rotation to coordinator BEFORE updating processor config
              camera_coord_.set_rotation(this->rotation_);
         }
         
         // Pass preview setting to coordinator to enable caching if needed
         camera_coord_.set_enable_preview(this->generate_preview_);
         
         camera_coord_.update_image_processor_config(
             spec.input_width, 
             spec.input_height, 
             spec.input_channels,
             processor_input_type,
             spec.normalize,
             spec.input_order
         );
         
         if (esp32_camera_utils_) {
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
              
              // Fix: Pass rotation as-is to support "fine rotation" (software)
              config.rotation = rotation_;
              
              // Helper: The underlying ImageProcessor/Rotator will handle 
              // 90/180/270 efficient paths or arbitrary software rotation.
              
              /* 
              switch(static_cast<int>(rotation_)) {
                  case 90:  config.rotation = esp32_camera_utils::ROTATION_90;  break;
                  case 180: config.rotation = esp32_camera_utils::ROTATION_180; break;
                  case 270: config.rotation = esp32_camera_utils::ROTATION_270; break;
                  default:  config.rotation = esp32_camera_utils::ROTATION_0;   break;
              }
              */
              
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
    // Validation is now handled by external component wrapped in coordinator
    // validation_coord_ is member, validator_ pointer is set via set_validator called by setup_priority
    
    // 4. Setup Camera Callback
    // Register callback on the global camera instance.
    
    // 5. Setup Flashlight Coordinator Callback
    flashlight_coord_.set_request_frame_callback([this](){
        this->frame_requested_ = true;
        this->last_request_time_ = millis();
        ESP_LOGD(TAG, "Frame requested via coordinator callback");
    });
    
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
  
    // 6. Setup Inference Object Pools
    INFERENCE_POOL_SIZE = calculate_optimal_pool_size();
    ESP_LOGI(TAG, "Auto-configured inference pool size: %zu entries", INFERENCE_POOL_SIZE);
    
    // Allocate pools
    inference_job_pool = std::make_unique<InferenceJob[]>(INFERENCE_POOL_SIZE);
    inference_result_pool = std::make_unique<InferenceResult[]>(INFERENCE_POOL_SIZE);
    inference_job_used = std::make_unique<bool[]>(INFERENCE_POOL_SIZE);
    inference_result_used = std::make_unique<bool[]>(INFERENCE_POOL_SIZE);
    
    // Initialize used flags
    for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
        inference_job_used[i] = false;
        inference_result_used[i] = false;
    }
    
    // 7. Setup Double Buffering Pipeline
    ESP_LOGI(TAG, "Initializing Double Buffering (Dual Core mode)...");
    input_queue_ = xQueueCreate(1, sizeof(InferenceJob*)); // Pointer depth 1 (Backpressure)
    output_queue_ = xQueueCreate(2, sizeof(InferenceResult*)); // Pointer depth 2
    
    if (!input_queue_ || !output_queue_) {
        ESP_LOGE(TAG, "Failed to create queues!");
        if (main_logs_) main_logs_->publish_state("CRITICAL: Failed to create queues!");
        mark_failed(); return;
    }
    
    BaseType_t res = xTaskCreatePinnedToCore(
        MeterReaderTFLite::inference_task, 
        "inference_task", 
        16384, // Stack size
        this, // Pass this instance
        1,    // Priority (Low)
        &inference_task_handle_, 
        0     // Pin to Core 0 (Main Loop is usually Core 1)
    );
    
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create inference task!");
        if (main_logs_) main_logs_->publish_state("CRITICAL: Failed to create inference task!");
        mark_failed(); return;
    }
    ESP_LOGI(TAG, "Double Buffering active on Core 0");
    #endif
   

    // Force Camera to Grayscale if using GRAY model (Workaround for YAML limit)
    // Only if user requested GRAYSCALE in config (we can't easily check the template sub here, 
    // but we know we just failed to set it in YAML).
    // Let's check the global pixel format config if feasible, or just force it if the model is V4 GRAY.
    // 8. Setup Dynamic Resource Buttons
    if (unload_button_) {
        unload_button_->add_on_press_callback([this]() { this->unload_resources(); });
    }
    if (reload_button_) {
        reload_button_->add_on_press_callback([this]() { this->reload_resources(); });
    }
}

void MeterReaderTFLite::set_camera(camera::Camera *camera) { // -> CameraCoord callback
    camera_coord_.set_camera((esp32_camera::ESP32Camera*)camera);
    // Register listener
    ESP_LOGD(TAG, "Registering CameraListener for MeterReaderTFLite");
    camera->add_listener(this); 
}

void MeterReaderTFLite::set_debug(bool debug) {
    this->debug_ = debug;
    this->tflite_coord_.set_debug(debug);
    this->camera_coord_.set_debug(debug);
    this->flashlight_coord_.set_debug(debug);
    
    // Also enable legacy debug mode if it exists but ideally we use debug_ everywhere
    #ifdef DEBUG_METER_READER_TFLITE
    this->set_debug_mode(debug);
    #endif
}

void MeterReaderTFLite::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
    if (frame_requested_.load() && !processing_frame_.load()) {
        pending_frame_ = image;
        pending_frame_acquisition_time_ = millis(); 
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
    // If paused, we still might want to update preview if enabled
    if (pause_processing_ && !generate_preview_ && !request_preview_) {
        ESP_LOGD(TAG, "Processing paused, skipping update");
        return;
    }

    // Calibration Cycle
    if (is_calibrating()) {
        // Calibration flash itself via inference results
        return; // Skip normal processing during calibration
    }
    
    // Setup Mode: Flash always on, ignore scheduling
    if (generate_preview_) {
        // Ensure flash is on (redundant check but safe) but do NOT run scheduler
        if (!flashlight_coord_.is_active()) {
             // In case it was turned off by something else, force it back
             flashlight_coord_.enable_flash();
        }
        
        // Trigger frame request for preview update 
        // (Similar to continuous mode but specifically for preview)
        if (!frame_available_ && !frame_requested_) {
             frame_requested_ = true;
             last_request_time_ = millis();
        } else if (frame_available_) {
             process_available_frame();
        }
        return; // Skip standard scheduling
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



void MeterReaderTFLite::loop() {
    if (this->is_failed()) return;

    // Watchdog: If frame requested but not arrived, reset state
    if (frame_requested_ && (millis() - last_request_time_ > frame_request_timeout_ms_)) {
        ESP_LOGE(TAG, "Frame request timed out (%u ms)! Camera Frame Failure.", frame_request_timeout_ms_);
        if (main_logs_) {
             char msg[100];
             snprintf(msg, sizeof(msg), "CRITICAL: Camera Frame Failure (Timeout %ums)", frame_request_timeout_ms_);
             main_logs_->publish_state(msg);
        }
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
            
            #ifdef DEBUG_METER_READER_MEMORY
            if (debug_memory_enabled_ && tensor_arena_used_sensor_) {
                  // tensor_arena_used_sensor_->publish_state(res_ptr->arena_used_bytes);
            }
            #endif

            if (total_inference_time_sensor_) {
                total_inference_time_sensor_->publish_state(millis() - res_ptr->total_start_time);
            }

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
                
                if (inference_logs_) {
                     // Publish to inference logs text sensor
                     char inference_log[150];
                     snprintf(inference_log, sizeof(inference_log),
                              "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)",
                              final_val, validated_val, valid ? "yes" : "no",
                              avg_conf * 100.0f, confidence_threshold_ * 100.0f);
                     inference_logs_->publish_state(inference_log);
                }

                // Build confidence list string
                std::string conf_list = "[";
                for (size_t i = 0; i < res_ptr->probabilities.size(); i++) {
                    if (i > 0) conf_list += ", ";
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%.3f", res_ptr->probabilities[i]);
                    conf_list += buf;
                }
                conf_list += "]";

                // Publish (matches process_full_image logic)
                if (valid && (validation_coord_.has_validator() || avg_conf >= confidence_threshold_)) {
                     ESP_LOGI(TAG, "Result: VALID (Raw: %.0f, Conf: %.3f, %s)", 
                              final_val, avg_conf, conf_list.c_str());
                     value_sensor_->publish_state(validated_val);
                     if (confidence_sensor_) {
                         confidence_sensor_->publish_state(avg_conf * 100.0f);
                     }
                } else {
                     ESP_LOGI(TAG, "Result: INVALID (Raw: %.0f, Conf: %.3f, %s)", 
                              final_val, avg_conf, conf_list.c_str());

                     #ifdef USE_DATA_COLLECTOR
                     // Detect suspicious patterns (e.g. all 0s or all 1s) using logic in ValueValidator
                     bool suspicious = false;
                     if (validation_coord_.has_validator()) {
                         suspicious = validation_coord_.get_validator()->is_hallucination_pattern(res_ptr->readings);
                     }

                     // Only collect if confidence is actually low OR if specific suspicious pattern detected OR if VALIDATION FAILED
                     // "Result: INVALID" implies !valid OR low confidence.
                     // We want to collect ALL invalid readings to catch High Confidence errors.
                     
                     if (true) { // Logic simplified: If we are in this block, it IS invalid.
                         ESP_LOGW(TAG, "Data Collection Triggered: Reading Invalid (Conf: %.1f%%, Suspicious: %s)", 
                                  avg_conf * 100.0f, suspicious ? "YES" : "NO");
                         trigger_low_confidence_collection(final_val, avg_conf);
                     }
                     #endif
                }
            }
            
            // Publish pool efficiency statistics
            #ifdef DEBUG_METER_READER_MEMORY
            static uint32_t last_pool_stats_publish = 0;
            if (millis() - last_pool_stats_publish > 60000) {  // Every 60 seconds
                last_pool_stats_publish = millis();
                
                if (pool_job_efficiency_sensor_) {
                    uint32_t hits = pool_job_hits.load();
                    uint32_t total = hits + pool_job_misses.load();
                    if (total > 0) {
                        pool_job_efficiency_sensor_->publish_state(100.0f * hits / total);
                    }
                }
                
                if (pool_result_efficiency_sensor_) {
                    uint32_t hits = pool_result_hits.load();
                    uint32_t total = hits + pool_result_misses.load();
                    if (total > 0) {
                        pool_result_efficiency_sensor_->publish_state(100.0f * hits / total);
                    }
                }
                
                // Publish arena efficiency
                if (arena_efficiency_sensor_) {
                    auto arena_stats = tflite_coord_.get_arena_stats();
                    arena_efficiency_sensor_->publish_state(arena_stats.efficiency);
                }
                
                // Publish heap fragmentation
                if (heap_fragmentation_sensor_) {
                    multi_heap_info_t info;
                    
                    // Check PSRAM first (if available), fallback to internal RAM
                    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                    if (free_psram > 0) {
                        // PSRAM available - measure PSRAM fragmentation
                        heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
                    } else {
                        // No PSRAM - measure internal RAM fragmentation
                        heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);
                    }
                    
                    // Calculate fragmentation: 100% - (largest_free_block / total_free * 100)
                    // Lower is better. 0% = no fragmentation, 100% = highly fragmented
                    float fragmentation = 0.0f;
                    if (info.total_free_bytes > 0) {
                        float efficiency = (float)info.largest_free_block / info.total_free_bytes;
                        fragmentation = (1.0f - efficiency) * 100.0f;
                    }
                    
                    heap_fragmentation_sensor_->publish_state(fragmentation);
                }
            }
            #endif
            
            // Cleanup
            free_inference_result(res_ptr);
            
            // Request next frame if configured.
            // Note: 'update()' triggers 'take_picture', so this maintains the loop if continuous mode is enabled.
        }
        processing_frame_ = false; // Release lock
    }
    #endif
}

#ifdef USE_DATA_COLLECTOR
void MeterReaderTFLite::trigger_low_confidence_collection(float value, float confidence) {
    if (!collect_low_confidence_ || !data_collector_) return;
    // User requested 0% to Threshold range.
    // if (confidence < low_confidence_trigger_threshold_) return; 
    if (this->collection_state_ != COLLECTION_IDLE) return; 

    // Prevent collection during Setup Mode (Preview) or calibration
    if (this->generate_preview_ || this->is_calibrating()) {
        ESP_LOGD(TAG, "Data collection skipped (Setup Mode/Calibration active)");
        return;
    } 

    ESP_LOGI(TAG, "Low confidence (%.2f%%) detected. Retaking with flash for data collection...", confidence * 100.0f);

    pending_collection_ = {value, confidence};
    this->collection_state_ = COLLECTION_WAITING_FOR_FLASH;
    
    // Force Flash via Coordinator
    flashlight_coord_.enable_flash();
    
    // Wait for stabilization
    uint32_t delay = flashlight_coord_.get_pre_time();
    // Safety clamp
    if (delay < 500) delay = 500;
    if (delay > 5000) delay = 5000;

    this->set_timeout(delay, [this]() {
        ESP_LOGD(TAG, "Flash stabilized. Requesting data collection frame.");
        this->collection_state_ = COLLECTION_WAITING_FOR_FRAME;
        this->frame_requested_ = true;
        this->last_request_time_ = millis();
    });
}
#endif

void MeterReaderTFLite::process_available_frame() {
    processing_frame_ = true;
    std::shared_ptr<camera::CameraImage> frame = pending_frame_;
    pending_frame_.reset();
    frame_available_ = false;
    
    // Data Collection Interception
    #ifdef USE_DATA_COLLECTOR
    if (this->collection_state_ == COLLECTION_WAITING_FOR_FRAME) {
         if (this->data_collector_) {
             this->data_collector_->collect_image(frame, camera_coord_.get_width(), camera_coord_.get_height(), camera_coord_.get_format(), pending_collection_.value, pending_collection_.confidence);
         }
         this->collection_state_ = COLLECTION_IDLE;
         this->flashlight_coord_.disable_flash();
         processing_frame_ = false;
         return; 
    }
    #endif
    
    if (frame) {
        process_full_image(frame);
    }
    processing_frame_ = false;
}

void MeterReaderTFLite::process_full_image(std::shared_ptr<camera::CameraImage> frame) {
    METER_DURATION_START("Total Processing");

    if (!tflite_coord_.is_model_loaded()) {
        ESP_LOGW(TAG, "Skipping frame - Model not loaded yet");
        return;
    }

    
    // Preview Logic (Rotation) - REMOVED: Now handled via cached image AFTER processing
    /*
    #if defined(DEV_ENABLE_ROTATION) && defined(USE_CAMERA_ROTATOR)
    if (generate_preview_ || request_preview_) {
         // ... implementation removed ...
    }
    #endif
    */



    // Check processing state
    bool pause = pause_processing_.load();
    bool preview_needed = generate_preview_ || request_preview_;

    if (pause && !preview_needed) {
        ESP_LOGI(TAG, "Setup Mode active & No Preview: Skipping.");
        return; 
    } else if (pause) {
        ESP_LOGI(TAG, "Setup Mode active: Processing for Preview only.");
    } else {
        ESP_LOGD(TAG, "Processing allowed: Starting inference.");
    }

    // Inference
    auto zones = crop_zone_handler_.get_zones();
    ESP_LOGI(TAG, "Processing Image: Found %d crop zones", zones.size());
    
    // Process frame -> buffers
    esphome::App.feed_wdt();
    
    uint32_t preprocess_start = millis();
    if (this->debug_) {
        ESP_LOGD(TAG, "Processing full image: Requesting frame processing from CameraCoordinator");
    }
    auto processed_buffers = camera_coord_.process_frame(frame, zones);
    
    // Check for debug image (last processed master)
    if (generate_preview_ || request_preview_) {
        auto preview = camera_coord_.get_debug_image();
        ESP_LOGD(TAG, "Checking for preview image. Ptr: %p", preview ? preview.get() : nullptr);
        
        if (preview) {
             // Draw crop zones on the preview image for debugging visualization.
             
             if (show_crop_areas_) {
                 #ifdef USE_CAMERA_DRAWING
                 #ifndef USE_HOST
                 // Cast to internal type removed. Rely on CameraImage interface and model config.
                 if (preview->get_data_buffer()) {
                     uint8_t* buf = preview->get_data_buffer();
                     
                     // Calculate dimensions based on current camera config and rotation
                     int w = camera_coord_.get_width();
                     int h = camera_coord_.get_height();
                     if (std::abs(rotation_ - 90.0f) < 0.1f || std::abs(rotation_ - 270.0f) < 0.1f) {
                         std::swap(w, h);
                     }
                     
                     uint16_t color = 0x07E0; // Light Green
                     
                     auto spec = tflite_coord_.get_model_spec();
                     int channels = spec.input_channels;
                     // RGB565 is 2 bytes but DrawingUtils usually works with pixels.
                     
                     // Simply skip drawing for now to avoid complexity or potential bugs, 
                     // OR rely on user verification that image is correct.
                     // Safe to modify 'preview' here because inference buffers were already copied/extracted
                     // in the `camera_coord_.process_frame` call above.
                     // The next frame will allocate a new buffer, so this modification does not affect future inference.
                 }
                 #endif
                 #endif
             }
             update_preview_image(preview);
        }
        if (request_preview_) request_preview_ = false;
    }
    
    #ifdef DEBUG_METER_READER_TIMING
    if (debug_timing_) {
        // Acquisition: Time from Request (last_request_time_) to Arrival (pending_frame_acquisition_time_)
        // Wait: Time from Arrival to Start of Processing (preprocess_start)
        ESP_LOGI(TAG, "Image Acquisition took %u ms", pending_frame_acquisition_time_ - last_request_time_);
        ESP_LOGI(TAG, "Preprocessing (Crop/Scale) took %u ms", millis() - preprocess_start);
    }
    #endif
    
    ESP_LOGI(TAG, "Processed Buffers Size: %d", processed_buffers.size());

    #ifdef SUPPORT_DOUBLE_BUFFERING
    ESP_LOGI(TAG, "Double Buffering IS ENABLED. Entering Async Path.");
    // Async Path
    if (processed_buffers.empty()) {
        ESP_LOGW(TAG, "Processed buffers empty. Skipping inference (Async).");
        processing_frame_ = false;
        return;
    }

    // If we are paused (Setup Mode/Preview Only), we stop here.
    if (pause_processing_) {
        processing_frame_ = false;
        return;
    }
    
    InferenceJob* job = allocate_inference_job();
    ESP_LOGI(TAG, "Allocated InferenceJob. Assigning resources...");

    job->frame = frame; // Keep alive
    job->crops = std::move(processed_buffers);
    // Use the start_time from METER_DURATION_START (defined at start of function)
    job->start_time = start_time; 
    
    ESP_LOGI(TAG, "Job prepared. Enqueuing... (Preprocessing took %u ms)", millis() - start_time);

    if (xQueueSend(input_queue_, &job, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Inference Queue Full - Dropping Frame");
        free_inference_job(job);
        processing_frame_ = false;
    } else {
        ESP_LOGI(TAG, "Job successfully enqueued.");
    }
    // Return immediately, loop() will handle result
    return;

    #endif
    
    // Buffers -> Inference (Synchronous Fallback)
    ESP_LOGI(TAG, "Entering Synchronous Inference Path (Single Core Fallback)");
    
    // Buffers -> Inference
    
    // Capture Peak Memory State *during* processing (buffers allocated)
    #ifdef DEBUG_METER_READER_MEMORY
    if (debug_memory_enabled_) {
 
        if (process_free_heap_sensor_) process_free_heap_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (process_free_psram_sensor_) process_free_psram_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    }
    #endif

    // Async Path (already checks empty above) but sync path needs it too
    if (processed_buffers.empty()) {
        ESP_LOGE(TAG, "No processed buffers generated (JPEG decode failed?). Skipping inference.");
        if (main_logs_) main_logs_->publish_state("ERROR: JPEG Decode Failed / No Buffers");
        processing_frame_ = false;
        return;
    }

    ESP_LOGI(TAG, "Context: %d buffers ready. Running synchronous inference...", processed_buffers.size());
    auto results = tflite_coord_.run_inference(processed_buffers);
    ESP_LOGI(TAG, "Synchronous inference complete. Results: %d", results.size());
    
    #ifdef DEBUG_METER_READER_MEMORY
    if (debug_memory_enabled_ && tensor_arena_used_sensor_) {
          // tensor_arena_used_sensor_->publish_state(tflite_coord_.get_arena_used_bytes());
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
        
        if (this->debug_) {
            ESP_LOGD(TAG, "Digit %d: Val=%.2f, Conf=%.2f, Success=%s", 
                     digit_index, res.value, res.confidence, res.success ? "YES" : "NO");
        }
        
        digit_index++;
    }
    
    if (!readings.empty()) {
        // Use helper to combine readings and log details (matches legacy behavior)
        float final_val = combine_readings(readings);

        float avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / confidences.size();
        
        float validated_val = final_val;
        bool valid = validate_and_update_reading(readings, confidences, validated_val);

        if (inference_logs_) {
             // Publish to inference logs text sensor
             char inference_log[150];
             snprintf(inference_log, sizeof(inference_log),
                      "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%% (high: %.1f%%))",
                      final_val, validated_val, valid ? "yes" : "no",
                      avg_conf * 100.0f, confidence_threshold_ * 100.0f, high_confidence_threshold_ * 100.0f);
             inference_logs_->publish_state(inference_log);
        }

        if (valid && (validation_coord_.has_validator() || avg_conf >= confidence_threshold_)) {
             // Removed checking of inference_log char buffer availability to match legacy cleanly
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%% (high: %.1f%%))", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f, high_confidence_threshold_ * 100.0f);
             
             if (value_sensor_) value_sensor_->publish_state(validated_val);
             if (confidence_sensor_) confidence_sensor_->publish_state(avg_conf * 100.0f);
             
             ESP_LOGI(TAG, "Reading published - valid and confidence threshold met");
        } else {
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%% (high: %.1f%%))", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f, high_confidence_threshold_ * 100.0f);
             ESP_LOGW(TAG, "Reading NOT published - %s", 
                     !valid ? "validation failed" : "confidence below threshold");
             
             #ifdef USE_DATA_COLLECTOR
             // Detect suspicious patterns (e.g. all 0s or all 1s) using logic in ValueValidator
             bool suspicious = false;
             if (validation_coord_.has_validator()) {
                 suspicious = validation_coord_.get_validator()->is_hallucination_pattern(readings);
             }

             // Only collect if confidence is actually low OR if specific suspicious pattern detected OR if VALIDATION FAILED
             // We are in the 'else' block of 'if (valid ...)', so we know it's invalid.
             
             if (true) {
                 ESP_LOGW(TAG, "Data Collection Triggered: Reading Invalid (Conf: %.1f%%, Suspicious: %s)", 
                          avg_conf * 100.0f, suspicious ? "YES" : "NO");
                 trigger_low_confidence_collection(final_val, avg_conf);
             }
             #endif
        }

        // Calibration Logic Hook
        // Calibration Logic Hook
        update_calibration(avg_conf);
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

void MeterReaderTFLite::set_last_valid_value(float value) {
    ESP_LOGI(TAG, "Manual override: Setting last valid value to %.1f", value);
    validation_coord_.set_last_valid_reading((int)value);
    
    // Immediately publish this as the truth to the sensor
    if (value_sensor_) {
        value_sensor_->publish_state(value);
    }
}

void MeterReaderTFLite::set_last_valid_value(const std::string &value) {
    // Try to parse just to publish to sensor if valid number
    char* end = nullptr;
    float f = strtof(value.c_str(), &end);
    if (end != value.c_str()) {
        if (value_sensor_) {
            value_sensor_->publish_state(f);
        }
    }

    // Pass string to validator to preserve digit count (leading zeros)
    // This allows solving the "8 digits vs 6 digits" mismatch by forcing 8 digits (with 00 prefix)
    validation_coord_.set_last_valid_reading(value);
}


void MeterReaderTFLite::set_flash_light(light::LightState* light) {
    flashlight_coord_.setup(this, light, nullptr);
}
void MeterReaderTFLite::set_flash_controller(flash_light_controller::FlashLightController* c) {
    flashlight_coord_.setup(this, nullptr, c);
}
void MeterReaderTFLite::set_generate_preview(bool generate) { 
    generate_preview_ = generate; 
    
    // Update Coordinator to enable/disable image caching dynamically
    camera_coord_.set_enable_preview(generate);
    
    // Refresh ImageProcessor config if model is loaded
    if (tflite_coord_.is_model_loaded()) {
         auto spec = tflite_coord_.get_model_spec();
         // Recalculate input type
         int processor_input_type = (spec.input_type == 1) ? 0 : 1;
         
         camera_coord_.update_image_processor_config(
             spec.input_width, spec.input_height, spec.input_channels,
             processor_input_type, spec.normalize, spec.input_order);
    }

    // Setup Mode Logic: Force light ON when preview is enabled
    if (generate) {
        ESP_LOGI(TAG, "Setup Mode (Preview) Enabled: Turning Flashlight ON");
        flashlight_coord_.enable_flash();
    } else {
        ESP_LOGI(TAG, "Setup Mode (Preview) Disabled: Turning Flashlight OFF");
        flashlight_coord_.disable_flash();
    }
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

#ifdef USE_WEB_SERVER
void MeterReaderTFLite::set_web_server(web_server_base::WebServerBase *web_server) {
    web_server_ = web_server;
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
    bool valid = validation_coord_.validate_reading(ival, conf, oval);
    val = static_cast<float>(oval);
    return valid;
}

bool MeterReaderTFLite::validate_and_update_reading(const std::vector<float>& digits, const std::vector<float>& confidences, float& val) {
    int oval = 0;
    bool valid = validation_coord_.validate_reading(digits, confidences, oval);
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
    // Only clear active flag if reset was successful.
    // Reset implies reverting to full frame configuration.
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
            
            ESP_LOGD(TAG, "Inference Task: Job received. Starting TFLite...");
            uint32_t start = millis();
            
            // Invoke TFLite (Thread-Safe because only this task calls it)
            // Note: queues are thread safe.
            auto tflite_results = self->tflite_coord_.run_inference(job->crops);
            
            InferenceResult* res = allocate_inference_result();
            res->inference_time = millis() - start;
            res->total_start_time = job->start_time;
            #ifdef DEBUG_METER_READER_MEMORY
            res->arena_used_bytes = self->tflite_coord_.get_arena_used_bytes();
            #endif
            res->success = true;
            
            for (auto& r : tflite_results) {
                res->readings.push_back(r.value);
                res->probabilities.push_back(r.confidence);
            }
            
            // Clean up job inputs
            free_inference_job(job);
            
            // Send back
            ESP_LOGD(TAG, "Inference Task: TFLite done in %u ms. Sending to output queue...", millis() - start);
            if (xQueueSend(self->output_queue_, &res, 100 / portTICK_PERIOD_MS) != pdTRUE) {
                // Queue push failed, likely due to main loop backpressure. Drop result to prevent blocking.
                free_inference_result(res);
            }
            job = nullptr;
        }
    }
}
#endif



void MeterReaderTFLite::start_flash_calibration() {
    if (!enable_flash_calibration_) {
        ESP_LOGW(TAG, "Flash calibration is disabled in config");
        if (main_logs_) main_logs_->publish_state("Flash calibration disabled");
        return;
    }
    ESP_LOGI(TAG, "Starting flash calibration...");
    if (main_logs_) main_logs_->publish_state("Starting flash calibration...");

    // Initialize/Reset Calibration State
    calibration_.state = FlashCalibrationHandler::CALIBRATING_PRE;
    calibration_.start_pre = CALIBRATION_START_PRE_MS; 
    calibration_.end_pre = CALIBRATION_END_PRE_MS;   
    calibration_.step_pre = CALIBRATION_STEP_PRE_MS;
    
    calibration_.start_post = CALIBRATION_START_POST_MS;
    calibration_.end_post = CALIBRATION_END_POST_MS;
    calibration_.step_post = CALIBRATION_STEP_POST_MS;

    calibration_.current_pre = calibration_.start_pre;
    calibration_.current_post = calibration_.start_post; // Start with safe/long post
    calibration_.baseline_confidence = 0.0f;
    calibration_.best_confidence = 0.0f;
    
    // Set initial timing
    flashlight_coord_.set_timing(calibration_.current_pre, calibration_.current_post);
    
    // Trigger first inference
    force_flash_inference();
}

void MeterReaderTFLite::update_calibration(float confidence) {
    if (!enable_flash_calibration_) return;
    if (!is_calibrating()) return;

    char log_msg[100];
    bool next_step = false;
    
    // 1. Establish Baseline (First run of Pre-Phase)
    if (calibration_.baseline_confidence == 0.0f) {
        calibration_.baseline_confidence = confidence;
        calibration_.best_pre = calibration_.current_pre;
        calibration_.best_post = calibration_.current_post;
        calibration_.best_confidence = confidence; 
        snprintf(log_msg, sizeof(log_msg), "Baseline Conf: %.1f%%. Testing Pre: %u", confidence * 100.0f, calibration_.current_pre);
        next_step = true;
    } else {
        // Check if worse
        if (confidence < (calibration_.baseline_confidence - 0.05f)) { // 5% drop threshold
            snprintf(log_msg, sizeof(log_msg), "Conf dropped to %.1f%% (Baseline %.1f%%). Step Failed.", confidence * 100.0f, calibration_.baseline_confidence * 100.0f);
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
                if (main_logs_) main_logs_->publish_state(log_msg);
                // Don't trigger next step
            }
        } else {
            // Good result, store as best and continue
            calibration_.best_confidence = confidence; 
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
    if (main_logs_) main_logs_->publish_state(log_msg);
    if (inference_logs_) inference_logs_->publish_state(log_msg);
    
    if (calibration_.state != FlashCalibrationHandler::FINISHED && next_step) {
        // Advance parameters
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
    }

    if (calibration_.state != FlashCalibrationHandler::FINISHED) { 
        flashlight_coord_.set_timing(calibration_.current_pre, calibration_.current_post); 
        this->set_timeout(1000, [this](){ 
            force_flash_inference(); 
        }); 
    } else { 
        // Finished, do final logging 
        char fin[100];
        snprintf(fin, sizeof(fin), "Done. Optimal: Pre=%ums, Post=%ums", calibration_.best_pre, calibration_.best_post);
        if (main_logs_) main_logs_->publish_state(fin);
        if (inference_logs_) inference_logs_->publish_state(fin);

        calibration_.state = FlashCalibrationHandler::IDLE; 
    }
}

void MeterReaderTFLite::unload_resources() {
    ESP_LOGI(TAG, "Unloading MeterReaderTFLite resources...");
    
    // 1. Pause processing
    pause_processing_.store(true);
    
    // 2. Stop Flashlight (stop timers/sequences)
    flashlight_coord_.disable_flash();
    
    // 3. Unload TFLite Model & Arena
    tflite_coord_.unload_model();
    
    // 4. Unload Camera ImageProcessor
    camera_coord_.unload();
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
    // 5. Free Object Pools
    // These are static unique_ptrs in this compilation unit
    inference_job_pool.reset();
    inference_result_pool.reset();
    inference_job_used.reset();
    inference_result_used.reset();
    #endif

    // 6. Force Heap Collection/Debug
    ESP_LOGI(TAG, "Resources unloaded. Heap status:");
    heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "  Internal Free: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  SPIRAM Free: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void MeterReaderTFLite::reload_resources() {
    ESP_LOGI(TAG, "Reloading MeterReaderTFLite resources...");
    
    #ifdef SUPPORT_DOUBLE_BUFFERING
    // 1. Re-allocate Pools
    INFERENCE_POOL_SIZE = calculate_optimal_pool_size();
    inference_job_pool = std::make_unique<InferenceJob[]>(INFERENCE_POOL_SIZE);
    inference_result_pool = std::make_unique<InferenceResult[]>(INFERENCE_POOL_SIZE);
    inference_job_used = std::make_unique<bool[]>(INFERENCE_POOL_SIZE);
    inference_result_used = std::make_unique<bool[]>(INFERENCE_POOL_SIZE);
    
    for (size_t i = 0; i < INFERENCE_POOL_SIZE; ++i) {
        inference_job_used[i] = false;
        inference_result_used[i] = false;
    }
    #endif

    // 2. Reload Model & Config
    // Use timeout to schedule on main loop tick to avoid stack issues/recursion
    this->set_timeout(100, [this]() {
         if (!tflite_coord_.load_model()) {
             ESP_LOGE(TAG, "Failed to reload model!");
             return;
         }
         
         // Update Camera Processor
         auto spec = tflite_coord_.get_model_spec();
         int processor_input_type = (spec.input_type == 1) ? 0 : 1;
         
         // Update Camera Coordinator
         camera_coord_.update_image_processor_config(
             spec.input_width, 
             spec.input_height, 
             spec.input_channels,
             processor_input_type,
             spec.normalize,
             spec.input_order
         );
         
         // Sync Utils if present
         if (esp32_camera_utils_) {
              esp32_camera_utils::ImageProcessorConfig config;
              config.camera_width = camera_coord_.get_width();
              config.camera_height = camera_coord_.get_height();
              
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
         
         // Resume processing
         pause_processing_.store(false);
         ESP_LOGI(TAG, "Resources reloaded and processing resumed.");
    });
}
