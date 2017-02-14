#include <boost/test/unit_test.hpp>
#include <rgbd_slam/Dummy.hpp>

using namespace rgbd_slam;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    rgbd_slam::DummyClass dummy;
    dummy.welcome();
}
