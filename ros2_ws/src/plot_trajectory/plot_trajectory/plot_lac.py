import matplotlib.pyplot as plt
import PIL.Image as Image
from math import cos, sin

lac_guerldedan = Image.open("guerledan.tiff")   


Xlim = [7146050, 7146750]
Ylim = [-534250, -533800]
scalex = 3000
scaley= 1000
offsetx = 0
offsety = -500
Xlim = [Xlim[0]-scalex+offsetx, Xlim[1]+scalex+offsetx]
Ylim = [Ylim[0]-scaley+offsety, Ylim[1]+scaley+offsety]
pointA = [7146100, -534100]
pointB = [7146300, -533900]
pointC = [7146600, -533900]
fig, ax = plt.subplots()
ax.imshow(lac_guerldedan, extent=[Xlim[0], Xlim[1], Ylim[0], Ylim[1]])
#ax.imshow(lac_guerldedan, extent=[7145550, 7147250,-536250, -531800])
ax.scatter(pointA[0], pointA[1], color="red", marker="o", label="Point A", s=100)
ax.scatter(pointB[0], pointB[1], color="red", marker="o", label="Point B")
ax.scatter(pointC[0], pointC[1], color="red", marker="o", label="Point C")
ax.set_xlim(7145550, 7147250)
ax.set_ylim(Ylim[0], Ylim[1])
# ax.scatter(x, y, color="blue", marker="x")
print("\n","PLotting...")
plt.pause(0.5)
print("\n","PLotting good")
plt.show()