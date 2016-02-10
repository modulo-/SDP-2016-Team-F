import numpy as np

colour_profiles = {
    "breadnel":
    {
        "red": {
            "min": np.array([138, 229, 0]),
            "max": np.array([255, 255, 255])
        },
        "yellow": {
            "min": np.array([20, 192, 192]),
            "max": np.array([40, 255, 255])
        },
        "blue": {
            'min': np.array([81, 110, 0]),
            'max': np.array([130, 255, 255])
            
        },
        "green": {
            'min': np.array([35, 159, 223]),
            'max': np.array([81, 255, 255])
        },
        "pink": {
            'min': np.array([121, 130, 220]),
            'max': np.array([194, 255, 255])
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
