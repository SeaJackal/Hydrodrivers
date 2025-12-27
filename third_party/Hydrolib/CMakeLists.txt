include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG bbef5d2bc8b0c6ccb8aa8bf298c905a339d106c9
)

FetchContent_MakeAvailable(Hydrolib)
