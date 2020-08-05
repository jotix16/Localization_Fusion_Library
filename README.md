# LocalizationFusionLibrary

# LocalizationFusionLibrary

#### Directory Structure
Parts of the code are organized as follows. Motivation is that we can develop them seperately.

```
src
└── motion_model                       # contains the base motion class and all its specific variations 
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt                  
└──kalman_filter                       # contains a FilterWraper, KalmanFilterBase and all filter variations
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt                  
└── meassurement                       # contains files related to measurements and their timekeeping
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt    
```
