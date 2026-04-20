#pragma once

#include "esphome/components/switch/switch.h"
#include "../sprinklify_controller.h"

namespace esphome {
namespace sprinklify {

class WinterModeSwitch : public switch_::Switch, public Component, public Parented<SprinklifyController> {
 public:
  void setup() override;
  void dump_config() override;
  WinterModeSwitch() = default;

 protected:
  void write_state(bool state) override;
};

}  // namespace sprinklify
}  // namespace esphome
