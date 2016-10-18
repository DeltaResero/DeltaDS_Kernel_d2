# Small script to update kernel with cm
git remote add upstream https://github.com/LineageOS/android_kernel_samsung_d2.git
git fetch upstream
git merge upstream/cm-14.1
