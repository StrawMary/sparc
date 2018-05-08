
# exemplu de rulat scriptul: ./facenet_usage.sh to_align_dir aligned_faces_dir temporary_aligned_people_folder ./classifiers/permanentClassifier.pkl

# scriptul asta va primi un folder cu poze ce trebuie aliniate (ale persoanei care trebuie adaugata in clasificator)
# mai primeste si pathul catre celelalte foldere aliniate 
# va mai primi si pathul spre unde sa copieze clasificatorul creat de facenet
# aliniaza pozele din primul director, le copiaza pe celelalte langa si reface clasificatorul
# copiaza calsificatorul unde trebuie
# copiaza si noile fete aliniate unde trebuie

# primul si al doile parametru
to_align_dir="$PWD/$1"
aligned_images_dir="$PWD/$2"
all_aligned_images_dir="$PWD/$3"
classifier_path="$PWD/$4"

#echo $to_align_dir
#echo $aligned_images_dir
#echo $classifier_path

# fac curatenie in folderele din facenet daca exista ceva (folderul input/output/clasificator)
input_dir="$PWD/Vision/FaceNet/input_dir/"
out_dir="$PWD/Vision/FaceNet/out_dir/" # asta nu cred ca imi trebuie
my_class_dir="$PWD/Vision/FaceNet/my_class/"
# accesez variabilele create: echo $myvariable $anothervar
rm -r $input_dir*
rm -r $out_dir*
rm -r $my_class_dir*

# copiaza folderul cu pozele persoanei de aliniat in cel input_dir
cp -r $to_align_dir/* $input_dir

# rulez align pe folderul ala
cd ./Vision/FaceNet/
python3 aligndata_first.py
cd ../../

# sterg fisierul txt ce se creaza dupa alignment
rm $out_dir/bounding_boxes*

# copiez folderul cu pozele persoanei deja aliniate in cel output_dir
cp -r $aligned_images_dir/* $out_dir

# acum out dir are doar un folder cu toate pozele aliniate ale noii persoane
# le copiez in fisierul de unde o sa le iau pe toate
cp -r $out_dir* $all_aligned_images_dir

# mut toate fetele aliniate (de la toate persoanele) in out_dir
cp -r $all_aligned_images_dir/* $out_dir

# fac clasificare pe toate, adica rulez scriptule ca sa obtin noul clasificator
cd ./Vision/FaceNet/
python3 create_classifier_se.py
cd ../../

# redenumesc fisierul cu clasificatorul si copiez clasificatorul obtinut unde trebuie
mv $my_class_dir/my_classifier.pkl $classifier_path
