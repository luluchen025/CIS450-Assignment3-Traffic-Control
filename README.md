# CIS450-Assignment3-Traffic-Control

## Project 3 – Traffic Control System using pthreads & Semaphores

**Course:** CIS 450  
**Student:** Lulu Chen  
**Environment:** Ubuntu 22.04 (Intel VM)  
**Language:** C (using pthreads, semaphores, condition variables)

---

## Overview

This project simulates a four-way intersection with stop signs using a multi-threaded approach.  
Each car is represented by a pthread and must safely:  
1. Arrive at the stop sign  
2. Stop for 2 seconds  
3. Cross the intersection  
4. Exit  

The implementation ensures:
- No collisions  
- No deadlocks  
- Fair access to the intersection  
- Same-direction “flow” (multiple cars from the same direction may cross sequentially)  
- Correct quadrant locking based on car movements  

---

## Folder Structure
```bash
p3/
│
├── tc.c # Main source code (final implementation with flow + HOL)
├── common.h # Provided helper functions (GetTime, Spin)
└── common_threads.h # Provided pthread wrappers (with error checking)
```
## How to Compile

In Ubuntu terminal:

```bash
gcc tc.c -o tc -pthread
./tc
```
