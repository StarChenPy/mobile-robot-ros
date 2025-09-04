from my_navigation_py.fixed_nav_service import call_fixed_navigation, NavigationResult


def jx_nav2() -> NavigationResult:
    nav_response = call_fixed_navigation()
    return nav_response