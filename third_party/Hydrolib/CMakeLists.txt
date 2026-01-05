include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG 84a4e358edeb0364f81e58edef76b341d2d23d55
)

FetchContent_MakeAvailable(Hydrolib)
