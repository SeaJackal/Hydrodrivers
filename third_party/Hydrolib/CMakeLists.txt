include(FetchContent)

FetchContent_Declare(
    Hydrolib
    GIT_REPOSITORY https://github.com/SeaJackal/Hydrolib-soft.git
    GIT_TAG 6bfda2dfd37fdae9d426152f862123ed681cfab2
)

FetchContent_MakeAvailable(Hydrolib)
