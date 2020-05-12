#include <gtest/gtest.h>

GTEST_API_ int main(int argc, char** argv)
{
	// GTest
	testing::InitGoogleTest(&argc, argv);
	auto ret = RUN_ALL_TESTS();
	system("pause");
	return ret;
}
