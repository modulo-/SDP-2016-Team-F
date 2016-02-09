import numpy as np

colour_profiles = {
    "breadnel":
    {
        "red": {
            "min": np.array([106, 150, 120]),
            "max": np.array([255, 255, 255])
        },
        "yellow": {
            "min": np.array([15, 54, 225]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
            'max': np.array([91, 255, 238]),
            'min': np.array([76, 86, 158])
        },
        "green": {
            'min': np.array([60, 66, 200]),
            'max': np.array([76, 130, 255])
        },
        "pink": {
            'min': np.array([121, 59, 219]),
            'max': np.array([170, 255, 255])
        }
    },
    "3d04_pc1":
    {
        "red": {
            "min": np.array([0, 148, 82]),
            "max": np.array([16, 255, 255])
        },
        "yellow": {
            "min": np.array([28, 112, 157]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
        	'min': np.array([70, 76, 101]),
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
    "3d04_pc4":
    {
        "red": {
            "min": np.array([106, 150, 120]),
            "max": np.array([255, 255, 255])
        },
        "yellow": {
            "min": np.array([15, 54, 225]),
            "max": np.array([42, 255, 255])
        },
        "blue": {
            'max': np.array([91, 255, 238]),
            'min': np.array([76, 86, 158])
        },
        "green": {
            'min': np.array([60, 66, 200]),
            'max': np.array([76, 130, 255])
        },
        "pink": {
            'min': np.array([121, 59, 219]),
            'max': np.array([170, 255, 255])
        }
    }
}
