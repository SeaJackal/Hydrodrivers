include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG c3bfce6dc89e4ffb6a24f7ae8b3e1779b4b3ed95
)

FetchContent_MakeAvailable(Hydrolib)
