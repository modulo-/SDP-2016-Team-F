import numpy as np
# 75 31 blue
# 32 69 green
# 62 80 pink
# 31 58 yellow
colour_profiles = {
    "pitch_3d04":
    {
        "red": {
            "min": np.array([177, 63, 100]),
            "max": np.array([192, 255, 255])
        },
        "yellow": {
            "min": np.array([26, 21, 230]),
            "max": np.array([35, 255, 255])
        },
        "blue": {
            'min': np.array([80, 60, 100]),
            'max': np.array([110, 255, 255])
            
        },
        "green": {
            'min': np.array([39, 16, 236]),
            'max': np.array([70, 255, 255])
        },
        "pink": {
            'min': np.array([150, 30, 200]),
            'max': np.array([180, 255, 255])
        }
    },
    "pitch_3d03":
    {
        "red": {
            "min": np.array([177, 148, 82]),
            "max": np.array([192, 255, 255])
        },
        "yellow": {
            "min": np.array([24, 112, 157]),
            "max": np.array([36, 255, 255])
        },
        "blue": {
            'min': np.array([80, 96, 101]),
            'max': np.array([110, 255, 255])
            
        },
        "green": {
            'min': np.array([40, 126, 102]),
            'max': np.array([70, 255, 255])
        },
        "pink": {
            'min': np.array([150, 85, 102]),
            'max': np.array([180, 222, 246])
        }
    }
}
