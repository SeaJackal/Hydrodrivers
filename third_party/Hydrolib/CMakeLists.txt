include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG eedadf7b27209d6dff44066805a895d5b3c12b98
)

FetchContent_MakeAvailable(Hydrolib)
