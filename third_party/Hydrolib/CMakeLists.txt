include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG f428300036ba1239ae56cf04187794c0cbef5e0d
)

FetchContent_MakeAvailable(Hydrolib)
