# Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -----------------------------------------------------------------------------
# LPC1768 device specific makefile.
# -----------------------------------------------------------------------------

# This version disables SD card, i.e. for direct flashing
DEFINES+=-DNO_SDCARD
LSCRIPT=$(BUILD_DIR)/../mbed/src/vendor/NXP/cmsis/LPC1768/GCC_ARM/LPC1768_NOSD.ld

include $(BUILD_DIR)/lpc1768.mk
