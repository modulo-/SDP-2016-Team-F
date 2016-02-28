import numpy as np
# 75 31 blue
# 32 69 green
# 62 80 pink
# 31 58 yellow
colour_profiles = {
    "pitch_3d04":
    {
        "red": {
            "min": np.array([170, 63, 100]),
            "max": np.array([180, 255, 255])
        },
        "yellow": {
            "min": np.array([24, 21, 230]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
            'min': np.array([80, 60, 100]),
            'max': np.array([107, 255, 255])
            
        },
        "green": {
            'min': np.array([50, 16, 236]),
            'max': np.array([65, 255, 255])
        },
        "pink": {
            'min': np.array([150, 30, 200]),
            'max': np.array([173, 255, 255])
        }
    },
    "pitch_3d03":
    {
        "red": {
            "min": np.array([0, 148, 82]),
            "max": np.array([16, 255, 255])
        },
        "yellow": {
            "min": np.array([28, 112, 157]),
            "max": np.array([39, 255, 255])
        },
        "blue": {
        	'min': np.array([80, 96, 101]),
            'max': np.array([107, 255, 255])
            
        },
        "green": {
            'min': np.array([41, 126, 102]),
            'max': np.array([55, 255, 255])
        },
        "pink": {
            'min': np.array([150, 85, 102]),
            'max': np.array([178, 222, 246])
        }
    }
}
