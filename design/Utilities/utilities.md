# Debug
In order to be able to debug possible errors this library makes use of a string-stream that outputs the debugg information in a certain file.
Since *FilterWrapper* is the top class in our class-hierarchy, it will too be the place where we define the output stream. All the other classes hold a pointer to it. 

## Definition of Debug Makros
Each class that intends to output debug messages should have the following members defined uniformly:
```cc
bool m_debug // bool variable which indicates whether debug is activated or no, possibly set by the build option
std::ofstream m_debug_stream // ostream used to output debug messages, defined in FilterWrapper class
std::ostream* m_debug_stream // pointer to the ostream, defined in any other class lower in the hierarchy than FilterWrapper
```

In order to realize an easy to use debug logging 2 Macros are defined in filter_utilities.h. 
```cc
#define DEBUG_W(msg) if (m_debug) { m_debug_stream << msg; } //used in FilterWrapper
#define DEBUG(msg) if (m_debug) { *m_debug_stream << msg; } // used everywhere else
```
## Debuging rules
The debuging messages should have the following form:
```cc
DEBUG("\n----- /ClassWeAreIn::FunctionWeAreIn" << " ------\n");
DEBUG("-- /Debuging message we want to log. --\n");
```
## TBD:
- Should we specify the debuging optin in the config file?
- Or should we log everytime the debug build option is set?