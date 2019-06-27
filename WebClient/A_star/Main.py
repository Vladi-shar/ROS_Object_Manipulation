import board
import search_tree
import json
import sys

init = json.loads(sys.argv[1])
target = json.loads(sys.argv[2])
ilen = len(init)
jlen = len(init[0])

b = board.Board(jlen, ilen)
a = board.Board(jlen, ilen)


for i in range(0, ilen):
	for j in range(0, jlen):
		if init[i][j] == 1:
			b.add_object(jlen - 1 - j, i)
		if target[i][j] == 1:
			a.add_object(jlen - 1 - j, i)

initial_state = search_tree.Search_node(b)
check_equal_board = lambda search_node : a == search_node
goal_test = check_equal_board
res = search_tree.Best_first_search(initial_state,goal_test,a)
print(str(res))
sys.stdout.flush()



# if __name__ == '__main__':
# 	main(matrices)

