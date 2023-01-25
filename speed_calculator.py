# speed finder function
def speedFinder(distance, takenTime):
    speed = distance / takenTime
    return speed


def averageFinder(completeList, averageOfItems):
    # finding the length of list.
    lengthOfList = len(completeList)

    # calculating the number items to find the average of
    selectedItems = lengthOfList - averageOfItems
    # 10 -6 =4

    # getting the list most recent items of list to find average of .
    selectedItemsList = completeList[selectedItems:]

    # finding the average .
    average = sum(selectedItemsList) / len(selectedItemsList)

    return average