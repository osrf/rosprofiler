string node
string host
string uri
string pid

# Statistics apply to this time window
time window_start
time window_stop

uint16 samples

uint16 threads

# Process CPU usage, as percentage of total local use
float32 cpu_load_mean
float32 cpu_load_std
float32 cpu_load_max

# Virtual Memory use 
float64 virt_mem_mean
float32 virt_mem_std
float64 virt_mem_max

# Real Memory Use
float64 real_mem_mean
float32 real_mem_std
float64 real_mem_max

