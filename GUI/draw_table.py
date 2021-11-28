import pygame
import random
import cv2
import os


width = 1280
height = 720
pygame.init()
screen = pygame.display.set_mode((width, height))
background = pygame.image.load("table.jpg")
avatar1 = pygame.image.load("avatar1.png")
avatar2 = pygame.image.load("avatar2.png")
deck = pygame.image.load("deck.jpg")
avatar1 = pygame.transform.scale(avatar1, (130, 130))
avatar2 = pygame.transform.scale(avatar2, (130, 130))
deck_img = pygame.transform.scale(deck, (130, 300))


def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = os.path.join(folder, filename)
        if img is not None:
            images.append(img)
    return images


def create_deck(dir):
    images = []
    stones = []
    for _ in range(28):
        img = pygame.image.load(dir[_])
        images.append(img)
        scaled_stone = pygame.transform.scale(images[_], (70, 130))
        stones.append(scaled_stone)
    return stones


def deal_players(d):
    p1_hand, p2_hand = [], []
    for _ in range(7):
        p1_hand.append(random.choice(d))
        d.remove(p1_hand[-1])
        p2_hand.append(random.choice(d))
        d.remove(p2_hand[-1])
        random.random()

    return p1_hand, p2_hand


# def rot_center(image, angle, x, y):
#     rotated_image = pygame.transform.rotate(image, angle)
#     new_rect = rotated_image.get_rect(center=image.get_rect(center=(x, y)).center)
#
#     return rotated_image, new_rect


def blitRotateCenter(surf, image, topleft, angle):

    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(center=image.get_rect(topleft=topleft).center)

    surf.blit(rotated_image, new_rect)


directories = load_images_from_folder("cards")
deck = create_deck(directories)
player1_hand, player2_hand = deal_players(deck)


choice = random.choice(player1_hand)
i = player1_hand.index(choice)

run = True
while run:

    screen.blit(background, (0, 0))
    screen.blit(avatar1, (300, 10))
    screen.blit(avatar2, (300, 580))
    screen.blit(deck_img, (1130, 170))


    # blit both players hands
    for card in range(7):
        screen.blit(player1_hand[card], (460 + 80 * card, 10))
        screen.blit(player2_hand[card], (460 + 80 * card, 580))


    position = choice.get_rect(center=(495+80*i, 74))

    for x in range(100):
        screen.blit(background, position, position)     # erase
        position = position.move(0, 2)                  # move player
        screen.blit(choice, position)                   # draw new player
        pygame.display.update()                         # and show it all
        pygame.time.delay(10)


    pygame.time.delay(5000)
    # blitRotateCenter(screen, choice, (495+80*i, 274), 90)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    pygame.display.update()
