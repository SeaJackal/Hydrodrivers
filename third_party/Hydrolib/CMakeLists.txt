include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG 3d196f9d4b9f3650070220f57173fe5d16b57aee
)

FetchContent_MakeAvailable(Hydrolib)
