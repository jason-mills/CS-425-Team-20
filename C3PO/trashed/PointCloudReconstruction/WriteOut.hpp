#ifndef _WRITEOUT_HPP_
#define _WRITEOUT_HPP_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>

namespace pt = boost::property_tree;
namespace xp = boost::property_tree::xml_parser;

class WriteOut {
private:
	pt::ptree tree;

public:
	// Constructors
	WriteOut() {}

	// Methods
	bool addPair(std::string key, std::string value) {
		tree.put(key, value);
	}
	
	bool toFile(std::string fname) {
		xp::write_xml(fname, tree);
		return true;
	}

};

#endif