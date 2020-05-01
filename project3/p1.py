def squreroot(number):
    if number is None:
        return 0

    if number < 0:
        return 0

    if number == 0 or number == 1:
        return number

    nums = list(range(1, number))
    index = number // 2

    while (nums[index] * nums[index]) > number:
        index //= 2

    while (nums[index] * nums[index]) < number:
        index += 1

    if (nums[index] * nums[index]) == number:
        return nums[index]
    else:
        return nums[index - 1]


print(squreroot(None))  # 0

print(squreroot(-7))  # 0

print(squreroot(0))  # 0

print(squreroot(1))  # 1

print(squreroot(9))  # 3

print(squreroot(16))  # 4

print(squreroot(27))  # 5

print("Pass" if (0 == squreroot(0)) else "Fail")

print("Pass" if (1 == squreroot(1)) else "Fail")

print("Pass" if (3 == squreroot(9)) else "Fail")

print("Pass" if (4 == squreroot(16)) else "Fail")

print("Pass" if (5 == squreroot(27)) else "Fail")
