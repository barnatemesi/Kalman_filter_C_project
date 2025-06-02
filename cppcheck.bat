REM Checking ./test.c and ./kalman_filter.c with static analyzer cppcheck
@echo on

cppcheck.exe -I inc/ -I helper_files/ -I kalman_filter/ -I data/ --enable=all --inconclusive --std=c11 --suppress=missingIncludeSystem kalman_filter/kalman_filter.c

cppcheck.exe -I inc/ -I helper_files/ -I kalman_filter/ -I data/ --enable=all --inconclusive --std=c11 --suppress=missingIncludeSystem test/test.c

pause