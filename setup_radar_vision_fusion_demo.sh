#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
cd $(dirname $0)
WGET="wget --proxy off"
URL_MODEL_10_0="http://software-dl.ti.com/jacinto7/esd/modelzoo/10_00_00/modelartifacts/AM62A/8bits/od-8200_onnxrt_coco_edgeai-mmdet_yolox_nano_lite_416x416_20220214_model_onnx.tar.gz"

# download and setup model artifacts
if [  -d /opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 ] ; then 
    echo "model is already downloaded"
else 
  if [  -f od-8220_onnxrt_coco_edgeai-od-8200_onnxrt_coco_edgeai-mmdet_yolox_nano_lite_416x416_20220214_model_onnx.tar.gz ] ; then 
    echo "model is already downloaded"
  else
    $WGET $URL_MODEL_10_0
    if [ "$?" -ne "0" ]; then
		  echo "Failed to download model; check proxy settings/environment variables. Alternatively, download the model on a PC and transfer to this directory"
    else
      mkdir /opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416
      tar -xf od-8220_onnxrt_coco_edgeai-od-8200_onnxrt_coco_edgeai-mmdet_yolox_nano_lite_416x416_20220214_model_onnx.tar.gz -C /opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 --warning=no-timestamp
    fi
  fi
fi

# install python dependencies
pip3 install -r requirements.txt