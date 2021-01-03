## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import os
import subprocess
from pathlib import Path

print(os.path.join("/home/christian/python_projects/image_data_pipeline/", "image_data"))

try:
    subprocess.check_call(["""docker run --rm --volume "/home/christian/python_projects/image_data_pipeline/image_data/:/input-output:rw" jjkka132/pngtoklg /bin/bash -c "/pngtoklg/png_to_klg/build/pngtoklg -w '/input-output/2020-12-17_12:28:19/' -o '/input-output/log2.klg'" """], shell=True)
except subprocess.CalledProcessError as error:
    print(error)

Path("/home/christian/python_projects/image_data_pipeline/image_data/2020-12-17_12:28:19/cad_data/").mkdir(parents=True, exist_ok=True)
os.rename("/home/christian/python_projects/image_data_pipeline/image_data/log2.klg" ,
          "/home/christian/python_projects/image_data_pipeline/image_data/2020-12-17_12:28:19/cad_data/log2.klg")
print ("finished")