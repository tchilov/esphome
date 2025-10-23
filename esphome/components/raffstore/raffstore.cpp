#include "raffstore.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#endif

namespace esphome {
namespace raffstore {

static const char *TAG = "raffstore.cover";

int tiltPositionAction;
#define TILPOS_IDLE 0
#define TILPOS_TILTING 1
#define TILPOS_POSITIONING 2

void Raffstore::dump_config() {
  LOG_COVER("", "Raffstore", this);
  ESP_LOGCONFIG(TAG,
                "  Open Duration: %.1fs\n"
                "  Close Duration: %.1fs\n"
                "  Full Tilt Duration: %.1fs",
                this->open_duration_ / 1e3f, this->close_duration_ / 1e3f, this->full_tilt_duration_ / 1e3f);
}

void Raffstore::setup() {
  // open duration shorter than it actually is will prevent the motor going into overcurrent
  // start fully open so to find its home the cover will close.
  // this way we will never hit the mechanical open limit
  this->position = esphome::cover::COVER_OPEN;
}

void Raffstore::loop() {
  if (this->current_operation == esphome::cover::COVER_OPERATION_IDLE)
    return;

  const uint32_t now = App.get_loop_component_start_time();

  // Recompute position every loop cycle
  this->recompute_position_();

  if (this->is_at_target_()) {
    if (this->has_built_in_endstop_ && (this->target_position_ == esphome::cover::COVER_OPEN ||
                                        this->target_position_ == esphome::cover::COVER_CLOSED)) {
      // Don't trigger stop, let the cover stop by itself.
      this->current_operation = esphome::cover::COVER_OPERATION_IDLE;
    } else {
      this->start_direction_(esphome::cover::COVER_OPERATION_IDLE);
    }
    this->publish_state();

    if (tiltPositionAction == TILPOS_TILTING) {
      this->tilt = this->target_tilt_;
    } else if (tiltPositionAction != TILPOS_TILTING && (this->tilt != this->target_tilt_)) {
      if (this->last_operation_ == esphome::cover::COVER_OPERATION_CLOSING) {
        this->start_direction_(esphome::cover::COVER_OPERATION_OPENING);
      } else if (this->last_operation_ == esphome::cover::COVER_OPERATION_OPENING) {
        this->start_direction_(esphome::cover::COVER_OPERATION_CLOSING);
      }

      tiltPositionAction = TILPOS_TILTING;
    } else {
      tiltPositionAction = TILPOS_IDLE;
    }
  }

  // Send current position every 100 ms
  if (now - this->last_publish_time_ > 100) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

float Raffstore::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits Raffstore::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(true);
  traits.set_supports_position(true);
  traits.set_supports_tilt(true);
  traits.set_supports_toggle(false);
  traits.set_supports_stop(true);

  return traits;
}

void Raffstore::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(esphome::cover::COVER_OPERATION_IDLE);
    this->publish_state();
  }

  if (call.get_toggle().has_value()) {
    if (this->current_operation != esphome::cover::COVER_OPERATION_IDLE) {
      this->start_direction_(esphome::cover::COVER_OPERATION_IDLE);
      this->publish_state();
    } else {
      if (this->position == esphome::cover::COVER_CLOSED ||
          this->last_operation_ == esphome::cover::COVER_OPERATION_CLOSING) {
        this->target_position_ = esphome::cover::COVER_OPEN;
        this->start_direction_(esphome::cover::COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = esphome::cover::COVER_CLOSED;
        this->start_direction_(esphome::cover::COVER_OPERATION_CLOSING);
      }
    }
  }

  if (call.get_tilt().has_value()) {
    auto tilt = *call.get_tilt();
    this->target_tilt_ = tilt;

    if (this->target_tilt_ > this->tilt) {
      tiltPositionAction = TILPOS_TILTING;
      this->start_direction_(esphome::cover::COVER_OPERATION_OPENING);
    } else if (this->target_tilt_ < this->tilt) {
      tiltPositionAction = TILPOS_TILTING;
      this->start_direction_(esphome::cover::COVER_OPERATION_CLOSING);
    }
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();

    if (pos == this->position) {
      // already at target
      if (this->manual_control_ && (pos == esphome::cover::COVER_OPEN || pos == esphome::cover::COVER_CLOSED)) {
        // for covers with manual control switch, we can't rely on the computed position, so if
        // the command triggered again, we'll assume it's in the opposite direction anyway.
        auto op = pos == esphome::cover::COVER_CLOSED ? esphome::cover::COVER_OPERATION_CLOSING
                                                      : esphome::cover::COVER_OPERATION_OPENING;
        this->position =
            pos == esphome::cover::COVER_CLOSED ? esphome::cover::COVER_OPEN : esphome::cover::COVER_CLOSED;
        this->target_position_ = pos;
        this->start_direction_(op);
      }
      // for covers with built in end stop, we should send the command again
      if (this->has_built_in_endstop_ && (pos == esphome::cover::COVER_OPEN || pos == esphome::cover::COVER_CLOSED)) {
        auto op = pos == esphome::cover::COVER_CLOSED ? esphome::cover::COVER_OPERATION_CLOSING
                                                      : esphome::cover::COVER_OPERATION_OPENING;
        this->target_position_ = pos;
        this->start_direction_(op);
      }
    } else {
      auto op =
          pos < this->position ? esphome::cover::COVER_OPERATION_CLOSING : esphome::cover::COVER_OPERATION_OPENING;

      if (this->manual_control_ && (pos == esphome::cover::COVER_OPEN || pos == esphome::cover::COVER_CLOSED))
        this->position =
            pos == esphome::cover::COVER_CLOSED ? esphome::cover::COVER_OPEN : esphome::cover::COVER_CLOSED;

      this->target_position_ = pos;
      this->start_direction_(op);
      tiltPositionAction = TILPOS_POSITIONING;
    }
  }
}

void Raffstore::stop_prev_trigger_() {
  if (this->prev_command_trigger_ != nullptr) {
    this->prev_command_trigger_->stop_action();
    this->prev_command_trigger_ = nullptr;
  }
}

bool Raffstore::is_at_target_() const {
  if (this->current_operation == esphome::cover::COVER_OPERATION_OPENING) {
    if (tiltPositionAction == TILPOS_POSITIONING)
      return this->position >= this->target_position_;
    else if (tiltPositionAction == TILPOS_TILTING)
      return this->tilt >= this->target_tilt_;
  } else if (this->current_operation == esphome::cover::COVER_OPERATION_CLOSING) {
    if (tiltPositionAction == TILPOS_POSITIONING)
      return this->position <= this->target_position_;
    else if (tiltPositionAction == TILPOS_TILTING)
      return this->tilt <= this->target_tilt_;
  }

  return true;
}

void Raffstore::start_direction_(esphome::cover::CoverOperation dir) {
  if (dir == this->current_operation && dir != esphome::cover::COVER_OPERATION_IDLE)
    return;

  this->recompute_position_();
  Trigger<> *trig;
  switch (dir) {
    case esphome::cover::COVER_OPERATION_IDLE:
      trig = this->stop_trigger_;
      break;
    case esphome::cover::COVER_OPERATION_OPENING:
      this->last_operation_ = dir;
      trig = this->open_trigger_;
      break;
    case esphome::cover::COVER_OPERATION_CLOSING:
      this->last_operation_ = dir;
      trig = this->close_trigger_;
      break;
    default:
      return;
  }

  this->current_operation = dir;

  const uint32_t now = millis();
  this->start_dir_time_ = now;
  this->last_recompute_time_ = now;

  this->stop_prev_trigger_();
  trig->trigger();
  this->prev_command_trigger_ = trig;
}

void Raffstore::recompute_position_() {
  if (this->current_operation == esphome::cover::COVER_OPERATION_IDLE)
    return;

  float dir;
  float action_dur;
  switch (this->current_operation) {
    case esphome::cover::COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case esphome::cover::COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();

  this->tilt += dir * (now - this->last_recompute_time_) / this->full_tilt_duration_;
  this->tilt = clamp(this->tilt, 0.0f, 1.0f);

  if (this->tilt == 0 or this->tilt == 1) {
    this->position += dir * (now - this->last_recompute_time_) / action_dur;
    this->position = clamp(this->position, 0.0f, 1.0f);
  }

  this->last_recompute_time_ = now;
}

}  // namespace raffstore
}  // namespace esphome
