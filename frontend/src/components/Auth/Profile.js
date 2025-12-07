import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../components/Auth/AuthProvider";
import { useHistory } from "@docusaurus/router";
import BrowserOnly from "@docusaurus/BrowserOnly"; // 🟢

const ProfileContent = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();

  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({
    technical_background: "",
    hardware_access: "",
    learning_goals: "",
  });
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (!loading && !user) {
      history.push("/");
    }
  }, [user, loading, history]);

  useEffect(() => {
    if (user) {
      setProfileData({
        technical_background: user.technical_background || "",
        hardware_access: user.hardware_access || "",
        learning_goals: user.learning_goals || "",
      });
    }
  }, [user]);

  // ... (Keep handleInputChange and handleSubmit functions same as before) ...
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setProfileData((prev) => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    const result = await updateProfile(profileData);
    if (result.success) {
      setMessage("Profile updated successfully!");
      setIsEditing(false);
    } else {
      setMessage(result.message || "Failed");
    }
  };
  // ... (End functions) ...

  if (loading) return <div>Loading...</div>;
  if (!user) return null;

  return (
    <div className="profile-container">
      <h2>User Profile</h2>
      {/* ... (Keep your JSX here) ... */}
      {/* For brevity, insert the JSX form from your previous Profile file here */}
      {!isEditing ? (
        <div className="profile-view">
          <p>
            <strong>Name:</strong> {user.name}
          </p>
          {/* ... rest of view ... */}
          <button className="btn-primary" onClick={() => setIsEditing(true)}>
            Edit
          </button>
        </div>
      ) : (
        <form onSubmit={handleSubmit}>
          {/* ... inputs ... */}
          <button type="submit" className="btn-primary">
            Save
          </button>
        </form>
      )}
    </div>
  );
};

const Profile = () => {
  return (
    <Layout title="My Profile">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => <ProfileContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default Profile;
