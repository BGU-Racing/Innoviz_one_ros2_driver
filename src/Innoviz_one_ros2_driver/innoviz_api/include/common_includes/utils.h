#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <string>
#include "common.h"

namespace invz {

	inline void CheckResult(Result& result)
	{
		/* In case of failure error_code should be different than 0 */
		if (result.error_code != invz::ERROR_CODE_OK)
		{
			/* TODO: change to string or stringstream */
			char error_code[8] = { '\0' };
			snprintf(error_code, 8, "%05d: ", result.error_code);
			std::string error_message = "ErrorCode " + std::string(error_code) + result.error_message;
			throw std::runtime_error(error_message);
		}
	}

	// generate 64 size string
	inline std::string Generate32ByteKey() {
		std::stringstream ss;
		for (size_t i = 0; i < 64; i++) {
			ss << "1";
		}
		return ss.str();
	}

	template<typename T>
	void normalize(T& s)
	{
		auto r = 1 / (hypot(s.x, hypot(s.y, s.z)));
		s.x *= r;
		s.y *= r;
		s.z *= r;
	}

}

#endif
