**Remove the vignetting phenomenon in real time**

1. Get the parameters from calibration.py in /src.
    - **output**: parameters1.txt
    - **run**: `python3 calibration.py "image_path"`

<br>

2. Modify a, b, and c in test.py with the parameters shown.
    - **output**: rqt_image_view
    - **run**: `rosrun vignetting_correation test.py`

<br>

![github_test](https://github.com/user-attachments/assets/09c7c86e-4980-4074-b1be-21d5bab0c9e7)
    
