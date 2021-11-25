import pygame
import random
from collections import Counter


# # INIT
# pygame.init()


# def playground_setup():
#     # window
#     screen = pygame.display.set_mode((1280, 800))
#
#     # background
#     bg = pygame.image.load("table.jpg")
#     screen.blit(bg, (0, 0))
#     # screen.fill((0, 0, 0))
#
#     # Title and Icon
#     pygame.display.set_caption("Domino")
#     icon = pygame.image.load("domino.bmp")
#     pygame.display.set_icon(icon)


def create_deck():
    template = [0, 1, 2, 3, 4, 5, 6]
    d = []
    for step in range(7):
        for i, t in enumerate(template):
            d.append([template[0], t])
        template.remove(template[0])

    ### print deck
    res = 0
    print("Initial deck:")
    for i in range(7, 0, -1):
        res += i
        print("\t\t\t\t", *d[res - i:res], end='\n')
    print('\n')

    return d


def deal_players(deck):
    p1_hand, p2_hand = [], []

    for _ in range(7):
        p1_hand.append(random.choice(deck))
        deck.remove(p1_hand[-1])
        p2_hand.append(random.choice(deck))
        deck.remove(p2_hand[-1])

    print("new deck:", deck)
    print("P1 hand:", p1_hand)
    print("P2 hand:", p2_hand, '\n')

    return p1_hand, p2_hand


def player1(deck, player1_hand, cards_on_table, play_options, possible_cards, chosen_card2):
    print("--------------------------Player 1---------------------------------")

    ### retrieving play options
    x = []
    gameover1 = False
    play_options.sort()
    chosen_card2.sort()
    x += play_options + chosen_card2
    play_options.clear()
    d_x = dict(Counter(x))

    if x[:2] == x[2:]:
        choice = random.choice(x[:2])
        play_options.append(choice)
        play_options.append(choice)
        random.random()
    else:
        for k, v in d_x.items():
            if v == 1 or v == 3:
                play_options.append(k)
    print("play options:", play_options, '\n')

    ### searching for possible cards to play
    for i in range(len(player1_hand)):
        if play_options[0] in player1_hand[i] or play_options[1] in player1_hand[i]:
            possible_cards.append(player1_hand[i])
        else:
            continue

    chosen_card1 = []
    if len(possible_cards) > 0:
        ### printing
        print("poss. cards:", possible_cards)
        chosen_card1 = random.choice(possible_cards)
        possible_cards.clear()

        print("Chosen card:", chosen_card1)
        cards_on_table.append(chosen_card1)
        player1_hand.remove(cards_on_table[-1])
        print("cards on table:", cards_on_table)
        print("p1 remaining hand:", player1_hand, '\n')

    elif len(possible_cards) == 0:
        print("-------------Player has to draw new stones!")
        while len(deck) >= 0:
            if len(deck) == 0:
                print("Deck is empty and no possible card to play!")
                gameover1 = True
                break


            player1_hand.append(random.choice(deck))
            deck.remove(player1_hand[-1])
            print("new hand:", player1_hand)
            print(len(deck))
            if play_options[0] in player1_hand[-1] or play_options[-1] in player1_hand[-1]:
                possible_cards.append(player1_hand[-1])

            if len(possible_cards) >= 1:
                print("poss. cards:", possible_cards)
                chosen_card1 = random.choice(possible_cards)
                possible_cards.clear()
                print("Chosen card:", chosen_card1)
                cards_on_table.append(chosen_card1)
                player1_hand.remove(cards_on_table[-1])
                print("cards on table:", cards_on_table)
                print("p1 remaining hand:", player1_hand, '\n')
                break



    return chosen_card1, gameover1


def player2(deck, player2_hand, cards_on_table, play_options, possible_cards, chosen_card1):
    print("--------------------------Player 2---------------------------------")

    ### retrieving play options
    x = []
    gameover2 = False
    play_options.sort()
    chosen_card1.sort()
    x += play_options + chosen_card1
    play_options.clear()
    d_x = dict(Counter(x))

    if x[:2] == x[2:] or (x[0] == x[1] and len(cards_on_table) == 1):
        # print("Singularity!")
        choice = random.choice(x[:2])
        play_options.append(choice)
        play_options.append(choice)
        random.random()
    else:
        for k, v in d_x.items():
            if v == 1 or v == 3:
                play_options.append(k)
    print("play options:", play_options, '\n')

    ### searching for possible cards to play
    for i in range(len(player2_hand)):
        if play_options[0] in player2_hand[i] or play_options[1] in player2_hand[i]:
            possible_cards.append(player2_hand[i])
        else:
            continue

    chosen_card2 = []
    if len(possible_cards) > 0:
        ### printing
        print("poss. cards:", possible_cards)
        chosen_card2 = random.choice(possible_cards)
        possible_cards.clear()
        print("Chosen card:", chosen_card2)
        cards_on_table.append(chosen_card2)
        player2_hand.remove(cards_on_table[-1])
        print("cards on table:", cards_on_table)
        print("p2 remaining hand:", player2_hand, '\n')

    elif len(possible_cards) == 0:
        print("-------------Player has to draw new stones!")
        while len(deck) >= 0:
            if len(deck) == 0:
                print("Deck is empty and no possible card to play!")
                gameover2 = True
                break

            player2_hand.append(random.choice(deck))
            deck.remove(player2_hand[-1])
            print("new hand:", player2_hand)
            print(len(deck))
            if play_options[0] in player2_hand[-1] or play_options[-1] in player2_hand[-1]:
                possible_cards.append(player2_hand[-1])

            if len(possible_cards) >= 1:
                print("poss. cards:", possible_cards)
                chosen_card2 = random.choice(possible_cards)
                possible_cards.clear()
                print("Chosen card:", chosen_card2)
                cards_on_table.append(chosen_card2)
                player2_hand.remove(cards_on_table[-1])
                print("cards on table:", cards_on_table)
                print("p1 remaining hand:", player2_hand, '\n')
                break




    return chosen_card2, gameover2


def game():
    deck = create_deck()
    player1_hand, player2_hand = deal_players(deck)

    ### round simulation
    cards_on_table = []
    play_options = []
    possible_cards = []
    gameover1, gameover2 = False, False

    ##### player 1 leading
    print("--------------------------Player 1---------------------------------")
    # cards_on_table+=[list(map(int, input().split()))]
    chosen_card1 = random.choice(player1_hand)
    print("Chosen card:", chosen_card1)
    cards_on_table.append(chosen_card1)
    player1_hand.remove(cards_on_table[-1])
    print("cards on table:", cards_on_table)
    print("p1 remaining hand:", player1_hand, '\n')


    while len(deck) > 0 or len(player1_hand) > 0 or len(player2_hand) > 0:

        chosen_card2, gameover2 = player2(deck, player2_hand, cards_on_table, play_options, possible_cards, chosen_card1)
        if len(player2_hand) == 0:
            print("Player 2 wins!!! CONGRATS")
            break
        elif gameover2:
            print("Player 1 wins!!!")
            break

        chosen_card1, gameover1 = player1(deck, player1_hand, cards_on_table, play_options, possible_cards, chosen_card2)
        if len(player1_hand) == 0 or gameover2:
            print("Player 1 wins!!! CONGRATS")
            break
        elif gameover1:
            print("Player 2 wins!!!")
            break
        #
        # if len(deck) == 0:
        #     break





if __name__ == '__main__':
    game()

# # Game loop
# running = True
# while running:
#
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#
#     # playground_setup()
#     # pygame.display.update()
