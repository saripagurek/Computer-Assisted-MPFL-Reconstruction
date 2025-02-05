LDFLAGS = -L. -lglfw -larmadillo -ldl # -lpthread
CXXFLAGS = -g -std=c++14 -stdlib=libc++ -Wall -Wno-write-strings -Wno-parentheses -Wno-unused-variable -Wno-unused-result -DMACOS #-pthread

vpath %.cpp ../src
vpath %.c   ../src/glad/src
vpath %.o   ../obj

OBJS = main.o gpuProgram.o linalg.o stl.o renderer.o separator.o anim.o csv.o gbuffer.o axes.o sphere.o tube.o drawSegs.o strokefont.o fg_stroke.o glad.o colourmap.o priority.o graph.o shortest.o spring.o

EXEC = mpfl

all:    $(EXEC)

$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(EXEC) $(OBJS) $(LDFLAGS) 

clean:
	rm -f  *~ $(EXEC) $(OBJS) Makefile.bak

depend:	
	makedepend -Y ../src/*.h ../src/*.cpp 2> /dev/null
	@echo ""
	@echo "Now edit the Makefile to remove the ../src/ prefix of all of the dependences for the .o files"
	@echo ""

# DO NOT DELETE

anim.o: ../src/main.h ../src/seq.h ../src/headers.h
anim.o: ../src/glad/include/glad/glad.h
anim.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
anim.o: ../src/sphere.h ../src/gpuProgram.h ../src/tube.h
anim.o: ../src/drawSegs.h ../src/separator.h ../src/stl.h
anim.o: ../src/priority.h ../src/anim.h ../src/colourmap.h
anim.o: ../src/strokefont.h ../src/shortest.h
axes.o: ../src/linalg.h ../src/gpuProgram.h ../src/headers.h
axes.o: ../src/glad/include/glad/glad.h
axes.o: ../src/glad/include/KHR/khrplatform.h ../src/seq.h
colourmap.o: ../src/headers.h ../src/glad/include/glad/glad.h
colourmap.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
drawSegs.o: ../src/headers.h ../src/glad/include/glad/glad.h
drawSegs.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
drawSegs.o: ../src/gpuProgram.h ../src/seq.h
fg_stroke.o: ../src/headers.h ../src/glad/include/glad/glad.h
fg_stroke.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
gpuProgram.o: ../src/headers.h ../src/glad/include/glad/glad.h
gpuProgram.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
gpuProgram.o: ../src/seq.h
graph.o: ../src/headers.h ../src/glad/include/glad/glad.h
graph.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
graph.o: ../src/seq.h
headers.o: ../src/glad/include/glad/glad.h
headers.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
main.o: ../src/seq.h ../src/headers.h ../src/glad/include/glad/glad.h
main.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
main.o: ../src/sphere.h ../src/gpuProgram.h ../src/tube.h
main.o: ../src/drawSegs.h ../src/separator.h ../src/stl.h
main.o: ../src/priority.h ../src/anim.h ../src/colourmap.h
main.o: ../src/strokefont.h ../src/shortest.h
renderer.o: ../src/stl.h ../src/linalg.h ../src/gpuProgram.h
renderer.o: ../src/headers.h ../src/glad/include/glad/glad.h
renderer.o: ../src/glad/include/KHR/khrplatform.h ../src/seq.h
renderer.o: ../src/gbuffer.h
separator.o: ../src/headers.h ../src/glad/include/glad/glad.h
separator.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
separator.o: ../src/stl.h ../src/gpuProgram.h ../src/seq.h
separator.o: ../src/priority.h
seq.o: ../src/headers.h ../src/glad/include/glad/glad.h
seq.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
shortest.o: ../src/headers.h ../src/glad/include/glad/glad.h
shortest.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
shortest.o: ../src/seq.h
sphere.o: ../src/linalg.h ../src/seq.h ../src/headers.h
sphere.o: ../src/glad/include/glad/glad.h
sphere.o: ../src/glad/include/KHR/khrplatform.h ../src/gpuProgram.h
stl.o: ../src/linalg.h ../src/gpuProgram.h ../src/headers.h
stl.o: ../src/glad/include/glad/glad.h
stl.o: ../src/glad/include/KHR/khrplatform.h ../src/seq.h
strokefont.o: ../src/headers.h ../src/glad/include/glad/glad.h
strokefont.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
strokefont.o: ../src/gpuProgram.h ../src/seq.h
tube.o: ../src/linalg.h ../src/seq.h ../src/headers.h
tube.o: ../src/glad/include/glad/glad.h
tube.o: ../src/glad/include/KHR/khrplatform.h ../src/gpuProgram.h
anim.o: ../src/headers.h ../src/glad/include/glad/glad.h
anim.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
anim.o: ../src/main.h ../src/seq.h ../src/sphere.h ../src/gpuProgram.h
anim.o: ../src/tube.h ../src/drawSegs.h ../src/separator.h
anim.o: ../src/stl.h ../src/priority.h ../src/anim.h
anim.o: ../src/colourmap.h ../src/strokefont.h ../src/shortest.h
axes.o: ../src/headers.h ../src/glad/include/glad/glad.h
axes.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
axes.o: ../src/axes.h ../src/gpuProgram.h ../src/seq.h
colourmap.o: ../src/colourmap.h ../src/headers.h
colourmap.o: ../src/glad/include/glad/glad.h
colourmap.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
csv.o: ../src/seq.h ../src/headers.h ../src/glad/include/glad/glad.h
csv.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
csv.o: ../src/csv.h
drawSegs.o: ../src/headers.h ../src/glad/include/glad/glad.h
drawSegs.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
drawSegs.o: ../src/drawSegs.h ../src/gpuProgram.h ../src/seq.h
fg_stroke.o: ../src/fg_stroke.h ../src/headers.h
fg_stroke.o: ../src/glad/include/glad/glad.h
fg_stroke.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
font.o: ../src/headers.h ../src/glad/include/glad/glad.h
font.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
gbuffer.o: ../src/headers.h ../src/glad/include/glad/glad.h
gbuffer.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
gbuffer.o: ../src/gbuffer.h ../src/font.h ../src/main.h ../src/seq.h
gbuffer.o: ../src/sphere.h ../src/gpuProgram.h ../src/tube.h
gbuffer.o: ../src/drawSegs.h ../src/separator.h ../src/stl.h
gbuffer.o: ../src/priority.h ../src/anim.h ../src/colourmap.h
gbuffer.o: ../src/strokefont.h ../src/shortest.h
gpuProgram.o: ../src/gpuProgram.h ../src/headers.h
gpuProgram.o: ../src/glad/include/glad/glad.h
gpuProgram.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
gpuProgram.o: ../src/seq.h
graph.o: ../src/graph.h ../src/headers.h
graph.o: ../src/glad/include/glad/glad.h
graph.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
graph.o: ../src/seq.h ../src/main.h ../src/sphere.h
graph.o: ../src/gpuProgram.h ../src/tube.h ../src/drawSegs.h
graph.o: ../src/separator.h ../src/stl.h ../src/priority.h
graph.o: ../src/anim.h ../src/colourmap.h ../src/strokefont.h
graph.o: ../src/shortest.h
linalg.o: ../src/linalg.h
main.o: ../src/headers.h ../src/glad/include/glad/glad.h
main.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
main.o: ../src/stl.h ../src/gpuProgram.h ../src/seq.h
main.o: ../src/renderer.h ../src/gbuffer.h ../src/main.h
main.o: ../src/sphere.h ../src/tube.h ../src/drawSegs.h
main.o: ../src/separator.h ../src/priority.h ../src/anim.h
main.o: ../src/colourmap.h ../src/strokefont.h ../src/shortest.h
main.o: ../src/font.h ../src/axes.h ../src/graph.h
priority.o: ../src/priority.h
renderer.o: ../src/headers.h ../src/glad/include/glad/glad.h
renderer.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
renderer.o: ../src/renderer.h ../src/stl.h ../src/gpuProgram.h
renderer.o: ../src/seq.h ../src/gbuffer.h ../src/main.h
renderer.o: ../src/sphere.h ../src/tube.h ../src/drawSegs.h
renderer.o: ../src/separator.h ../src/priority.h ../src/anim.h
renderer.o: ../src/colourmap.h ../src/strokefont.h ../src/shortest.h
separator-with-qvoronoi.o: ../src/separator.h ../src/headers.h
separator-with-qvoronoi.o: ../src/glad/include/glad/glad.h
separator-with-qvoronoi.o: ../src/glad/include/KHR/khrplatform.h
separator-with-qvoronoi.o: ../src/linalg.h ../src/stl.h
separator-with-qvoronoi.o: ../src/gpuProgram.h ../src/seq.h
separator-with-qvoronoi.o: ../src/priority.h ../src/main.h
separator-with-qvoronoi.o: ../src/sphere.h ../src/tube.h
separator-with-qvoronoi.o: ../src/drawSegs.h ../src/anim.h
separator-with-qvoronoi.o: ../src/colourmap.h ../src/strokefont.h
separator-with-qvoronoi.o: ../src/shortest.h
separator.o: ../src/separator.h ../src/headers.h
separator.o: ../src/glad/include/glad/glad.h
separator.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
separator.o: ../src/stl.h ../src/gpuProgram.h ../src/seq.h
separator.o: ../src/priority.h ../src/main.h ../src/sphere.h
separator.o: ../src/tube.h ../src/drawSegs.h ../src/anim.h
separator.o: ../src/colourmap.h ../src/strokefont.h ../src/shortest.h
separator.o: ../src/priority.cpp
shortest.o: ../src/main.h ../src/seq.h ../src/headers.h
shortest.o: ../src/glad/include/glad/glad.h
shortest.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
shortest.o: ../src/sphere.h ../src/gpuProgram.h ../src/tube.h
shortest.o: ../src/drawSegs.h ../src/separator.h ../src/stl.h
shortest.o: ../src/priority.h ../src/anim.h ../src/colourmap.h
shortest.o: ../src/strokefont.h ../src/shortest.h
showstl.o: ../src/headers.h ../src/glad/include/glad/glad.h
showstl.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
sphere.o: ../src/sphere.h ../src/linalg.h ../src/seq.h
sphere.o: ../src/headers.h ../src/glad/include/glad/glad.h
sphere.o: ../src/glad/include/KHR/khrplatform.h ../src/gpuProgram.h
stl.o: ../src/headers.h ../src/glad/include/glad/glad.h
stl.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
stl.o: ../src/stl.h ../src/gpuProgram.h ../src/seq.h
strokefont.o: ../src/strokefont.h ../src/headers.h
strokefont.o: ../src/glad/include/glad/glad.h
strokefont.o: ../src/glad/include/KHR/khrplatform.h ../src/linalg.h
strokefont.o: ../src/gpuProgram.h ../src/seq.h ../src/fg_stroke.h
tube.o: ../src/tube.h ../src/linalg.h ../src/seq.h ../src/headers.h
tube.o: ../src/glad/include/glad/glad.h
tube.o: ../src/glad/include/KHR/khrplatform.h ../src/gpuProgram.h
spring.o: ../src/spring.h ../src/linalg.h ../src/seq.h ../src/headers.h 
spring.o: ../src/glad/include/glad/glad.h ../src/glad/include/KHR/khrplatform.h ../src/gpuProgram.h
