include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG c6c795b2c8c9edecf4cfd8097ef3d4ba8e1c3aac
)

FetchContent_MakeAvailable(Hydrolib)
