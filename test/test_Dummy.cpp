#include <boost/test/unit_test.hpp>
#include <exploration/Dummy.hpp>

using namespace exploration;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    exploration::DummyClass dummy;
    dummy.welcome();
}
