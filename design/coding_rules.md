#Coding Rules
Aim of this document is to fix guidlines how to code inside this module. In a later state we can define a clang format to force our defined rules. 

##Rules
###Class names
Classes should be written in CamelCase with a capitalized letter at start. There should not be used any prefix. Class names should be nouns.
```cpp
class ExampleClass{...}
```
Base classes of inheritance must have the postfix `Base`
```cpp
class ExampleClassBase{...}
```
###Variable names
Variables should be written in snake_case with short letter at start.
```cpp
VariableType variable_name;
```
Member variables of classes should be written with a prefix `m_`
```cpp
class ExampleClass{
    int m_number;
}
```
Postfixes for pointer types and iterators are recommended but optional?
```cpp
VariableType* variable_name_ptr;
VariableType::itator variable_name_it;
```
###Function names
Names of free or member function should be written in snake_case with a short letter at start. The name has to be a verb. 
```cpp
class ExampleClass{
    int m_number;
    int show_number();
}
```
###Function paramters
The same rules for the paramters apply than for variable names. Specifier like `const`, `&` and `&&` have to be written in the following order. 
```cpp
int show_example(const ParameterType & parameter_name) const;
```
###Namespaces
Namespaces have to be used for this project and should be formated in the following style.
```cpp
namespace iav
{
namespace localization
{

...

} // namespace localization
} // iav

```

###Include guards
In the first instance I would suggest to just use `#pragma once`.

###Documentation style
I suggest doxygen style.
```cpp
/**
* This class shows an example documentation.
*/
class ExampleClass{
public:
    /**
    * Show internal number.
    * @param[in] input_variable Some input
    * @param[out] output_variable Some outout
    * @return The internal number.
    int show_number(int input_variable, int & output_variable);
    */
private:
    int m_number; //< Internal number
}
```