include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG 378273ce0f70ef69073624e4423ac02c720c485f
)

FetchContent_MakeAvailable(Hydrolib)
