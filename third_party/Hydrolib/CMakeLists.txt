include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG bea0c9fd6c4f287fe660626eb1a97a4c065900a0
)

FetchContent_MakeAvailable(Hydrolib)
