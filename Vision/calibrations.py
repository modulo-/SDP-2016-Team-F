import numpy as np
# 75 31 blue
# 32 69 green
# 62 80 pink
# 31 58 yellow
colour_profiles = {
    "breadnel":
    {
        "red": {
            "min": np.array([174, 63, 100]),
            "max": np.array([255, 255, 255])
        },
        "yellow": {
            "min": np.array([28, 21, 230]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
            'min': np.array([85, 60, 100]),
            'max': np.array([107, 255, 255])
            
        },
        "green": {
            'min': np.array([51, 16, 236]),
            'max': np.array([73, 255, 255])
        },
        "pink": {
            'min': np.array([150, 30, 200]),
            'max': np.array([170, 255, 255])
        }
    },
    "3d04_pc1":
    {
        "red": {
            "min": np.array([174, 148, 82]),
            "max": np.array([255, 255, 255])
        },
        "yellow": {
            "min": np.array([28, 112, 157]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
        	'min': np.array([70, 96, 101]),
            'max': np.array([141, 255, 255])
            
        },
        "green": {
            'min': np.array([50, 126, 102]),
            'max': np.array([60, 255, 255])
        },
        "pink": {
            'min': np.array([160, 85, 102]),
            'max': np.array([200, 222, 246])
        }
    },
    "3d04_pc3":
    {
        "red": {
            "min": np.array([0, 150, 214]),
            "max": np.array([16, 255, 255])
        },
        "yellow": {
            "min": np.array([28, 148, 208]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
            'min': np.array([70, 130, 150]),
            'max': np.array([141, 255, 255])
            
        },
        "green": {
            'min': np.array([50, 183, 183]),
            'max': np.array([60, 255, 255])
        },
        "pink": {
            'min': np.array([160, 125, 147]),
            'max': np.array([200, 255, 255])
        }
    },
    "3d04_pc4":
    {
        "red": {
            "min": np.array([0, 178, 148]),
            "max": np.array([5, 255, 255])
        },
        "yellow": {
            "min": np.array([21, 172, 181]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
        	'min': np.array([85, 79, 116]),
            'max': np.array([128, 255, 255])
            
        },
        "green": {
            'min': np.array([50, 180, 151]),
            'max': np.array([65, 255, 255])
        },
        "pink": {
            'min': np.array([115, 124, 76]),
            'max': np.array([190, 255, 255])
        }
    }
}
